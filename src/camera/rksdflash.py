#!/usr/bin/env python3
"""
rksdflash.py -- write a Rockchip (RK350? / Luckfox Pico/Lyra) MicroSD image bundle
to an SD card on Linux, without SocToolKit / SDDiskTool.

    sudo ./rksdflash.py <image-dir> <device>
    sudo ./rksdflash.py Ubuntu_Luckfox_Lyra_MicroSD_250417 /dev/sdX

It does what the Windows tool does:

  1. parses parameter.txt (the mtdparts CMDLINE + any "uuid:name=..." lines)
  2. builds the boot ID block from MiniLoaderAll.bin and writes it at LBA 64
  3. writes a GPT that matches parameter.txt, byte-compatible with
     `rkdeveloptool gpt parameter.txt` (same partition names, PARTUUIDs,
     bootable flag, and "grow" handling)
  4. writes each <partition>.img at the offset parameter.txt gives it
  5. reads everything back and verifies it

No external tools or Python packages are needed (no sgdisk, no simg2img,
no rkdeveloptool). Android-sparse .img files are expanded on the fly.

The ID block builder is a port of rkdeveloptool's UpgradeLoader() /
MakeIDBlockData(); its output was checked byte-for-byte against Rockchip's
own boot_merger for the RK3506 loader.
"""

import argparse
import ctypes
import fcntl
import hashlib
import os
import re
import stat
import struct
import subprocess
import sys
import tempfile

SECTOR = 512
GPT_ENTRIES = 128
GPT_ENTRY_SIZE = 128
IDB_LBA = 64                      # where the BootROM looks for the ID block
PART_PROPERTY_BOOTABLE = 1 << 2
SPARSE_MAGIC = 0xED26FF3A


# ----------------------------------------------------------------- utilities

class Fail(Exception):
    pass


def log(msg=""):
    print(msg, flush=True)


def human(n):
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if abs(n) < 1024 or unit == "TiB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{n} B"
        n /= 1024.0


def align(value, boundary):
    return (value + boundary - 1) // boundary * boundary


# --------------------------------------------------------- Rockchip RC4 / CRC

_RC4_KEY = bytes([124, 78, 3, 4, 85, 5, 9, 7, 45, 44, 123, 56, 23, 13, 23, 17])


def rc4(buf):
    """Rockchip's P_RC4(). It is its own inverse."""
    S = list(range(256))
    K = [_RC4_KEY[i & 0x0F] for i in range(256)]
    j = 0
    for i in range(256):
        j = (j + S[i] + K[i]) & 0xFF
        S[i], S[j] = S[j], S[i]
    out = bytearray(buf)
    i = j = 0
    for x in range(len(out)):
        i = (i + 1) & 0xFF
        j = (j + S[i]) & 0xFF
        S[i], S[j] = S[j], S[i]
        out[x] ^= S[(S[i] + S[j]) & 0xFF]
    return bytes(out)


def rc4_per_sector(data):
    return b"".join(rc4(data[o:o + SECTOR]) for o in range(0, len(data), SECTOR))


def _crc32_table(poly):
    table = []
    for i in range(256):
        acc = i << 24
        for _ in range(8):
            acc = ((acc << 1) ^ poly) & 0xFFFFFFFF if acc & 0x80000000 else (acc << 1) & 0xFFFFFFFF
        table.append(acc)
    return table


_CRC32_RK = _crc32_table(0x04C10DB7)


def crc32_rk(data):
    """Rockchip's CRC_32 (poly 0x04C10DB7, MSB-first, init 0). Legacy IDB only."""
    acc = 0
    for b in data:
        acc = ((acc << 8) & 0xFFFFFFFF) ^ _CRC32_RK[((acc >> 24) ^ b) & 0xFF]
    return acc


def _crc16_table(poly=0x1021):
    table = []
    for i in range(256):
        data = i << 8
        acc = 0
        for _ in range(8):
            acc = ((acc << 1) ^ poly) & 0xFFFF if (data ^ acc) & 0x8000 else (acc << 1) & 0xFFFF
            data = (data << 1) & 0xFFFF
        table.append(acc)
    return table


_CRC16 = _crc16_table()


def crc16_rk(data):
    acc = 0
    for b in data:
        acc = ((acc << 8) & 0xFFFF) ^ _CRC16[((acc >> 8) ^ b) & 0xFF]
    return acc


def crc32_le(data, crc=0):
    """Standard CRC-32 (poly 0xEDB88320), used for the GPT."""
    import zlib
    return zlib.crc32(data, crc) & 0xFFFFFFFF


# ------------------------------------------------------------ parameter.txt

class Partition:
    def __init__(self, name, offset, size, flags):
        self.name = name        # "rootfs"
        self.offset = offset    # in 512-byte sectors
        self.size = size        # in sectors, or None when it should grow
        self.flags = flags      # {"grow", "bootable", ...}
        self.uuid = None        # 16 raw GPT bytes
        self.image = None       # path to <name>.img

    @property
    def grow(self):
        return self.size is None

    def __repr__(self):
        size = "grow" if self.grow else f"{self.size} sectors"
        return f"<{self.name} @{self.offset} {size}>"


def uuid_to_gpt_bytes(text):
    """Same conversion as rkdeveloptool's string_to_uuid(): plain hex, then the
    first three fields byte-swapped -- i.e. a normal mixed-endian GPT GUID, so
    Linux reports exactly the PARTUUID written in parameter.txt."""
    hexstr = text.replace("-", "").strip()
    if len(hexstr) != 32:
        raise Fail(f"bad uuid in parameter.txt: {text!r}")
    raw = bytes.fromhex(hexstr)
    return (raw[0:4][::-1] + raw[4:6][::-1] + raw[6:8][::-1] + raw[8:16])


def gpt_bytes_to_uuid(raw):
    b = raw[0:4][::-1] + raw[4:6][::-1] + raw[6:8][::-1] + raw[8:16]
    h = b.hex()
    return f"{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}"


def parse_parameter(path):
    partitions, uuids, meta = [], {}, {}
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            m = re.search(r"uuid:\s*([^=]+)=\s*(\S+)", line)
            if m:
                uuids[m.group(1).strip()] = uuid_to_gpt_bytes(m.group(2))
                continue

            if "mtdparts" not in line:
                if ":" in line:
                    k, _, v = line.partition(":")
                    meta[k.strip()] = v.strip()
                continue

            # keep everything after the colon that follows "mtdparts=<device>"
            colon = line.find(":", line.find("mtdparts"))
            if colon == -1:
                continue
            spec = line[colon + 1:]
            for chunk in spec.split(","):
                m = re.match(r"\s*(-|0x[0-9a-fA-F]+)@(0x[0-9a-fA-F]+)\(([^)]+)\)", chunk)
                if not m:
                    continue
                size = None if m.group(1) == "-" else int(m.group(1), 16)
                offset = int(m.group(2), 16)
                bits = m.group(3).split(":")
                name = bits[0].strip()
                flags = {f.strip() for f in bits[1:]}
                if "grow" in flags:
                    size = None
                partitions.append(Partition(name, offset, size, flags))

    if not partitions:
        raise Fail(f"no mtdparts partition list found in {path}")

    for part in partitions:
        part.uuid = uuids.get(part.name)
    return partitions, meta


# ------------------------------------------------- MiniLoaderAll.bin -> IDB

RKBOOT_TAGS = (0x544F4F42, 0x2052444C)      # 'BOOT', 'LDR '
_HDR = "<IHII HBBBBB I BIB BIB BIB BB"
_HDR_FIELDS = ("tag size version merge year month day hour minute second chip "
               "n471 off471 esz471 n472 off472 esz472 nldr offldr eszldr "
               "sign rc4").split()


def parse_rkboot(blob):
    if len(blob) < struct.calcsize(_HDR):
        raise Fail("loader file is too small to be a Rockchip BOOT/LDR image")
    head = dict(zip(_HDR_FIELDS, struct.unpack_from(_HDR, blob, 0)))
    if head["tag"] not in RKBOOT_TAGS:
        raise Fail("not a Rockchip loader (missing 'BOOT'/'LDR ' tag)")
    if head["size"] != 102:
        raise Fail(f"unexpected RKBOOT header size {head['size']}")

    entries = {}
    for off, count, esize in ((head["off471"], head["n471"], head["esz471"]),
                              (head["off472"], head["n472"], head["esz472"]),
                              (head["offldr"], head["nldr"], head["eszldr"])):
        for i in range(count):
            rec = blob[off + i * esize: off + (i + 1) * esize]
            if len(rec) < 57:
                raise Fail("truncated loader entry table")
            etype = struct.unpack_from("<I", rec, 1)[0]
            name = rec[5:45].decode("utf-16-le", "replace").split("\x00")[0]
            doff, dsize, _delay = struct.unpack_from("<III", rec, 45)
            if doff + dsize > len(blob):
                raise Fail(f"loader entry {name!r} runs past end of file")
            entries.setdefault(etype, {})[name] = blob[doff:doff + dsize]
    return head, entries


def make_legacy_idb(flash_data, flash_boot, rc4_off):
    """Port of rkdeveloptool MakeIDBlockData() -- pre-RK3568 style ID block."""
    data_sec = align(len(flash_data), 2048) // SECTOR
    boot_sec = align(len(flash_boot), 2048) // SECTOR

    sec0 = bytearray(SECTOR)
    struct.pack_into("<I4xIHH", sec0, 0, 0x0FF0AA55, 1 if rc4_off else 0, 4, 4)
    struct.pack_into("<HH", sec0, 506, data_sec, data_sec + boot_sec)

    sec1 = bytearray(SECTOR)
    struct.pack_into("<HH", sec1, 0, 0x000C, 0xFFFF)
    struct.pack_into("<I", sec1, 10, 0x38324B52)       # 'RK28'

    # sector 2 layout: usInfoSize@0, bChipInfo[16]@2, reserved[473]@18,
    # szVcTag[3]@491, usSec0Crc@494, usSec1Crc@496, uiBootCodeCrc@498,
    # usSec3CustomDataOffset@502, usSec3CustomDataSize@504, szCrcTag[4]@506,
    # usSec3Crc@510
    sec2 = bytearray(SECTOR)
    sec2[491:494] = b"VC\x00"
    sec2[506:510] = b"CRC\x00"

    sec3 = bytearray(SECTOR)

    struct.pack_into("<H", sec2, 494, crc16_rk(sec0))
    struct.pack_into("<H", sec2, 496, crc16_rk(sec1))
    struct.pack_into("<H", sec2, 510, crc16_rk(sec3))

    if rc4_off:
        flash_data = rc4_per_sector(flash_data)
        flash_boot = rc4_per_sector(flash_boot)

    idb = bytearray((4 + data_sec + boot_sec) * SECTOR)
    idb[0:SECTOR] = sec0
    idb[SECTOR:2 * SECTOR] = sec1
    idb[3 * SECTOR:4 * SECTOR] = sec3
    idb[4 * SECTOR:4 * SECTOR + len(flash_data)] = flash_data
    off = (4 + data_sec) * SECTOR
    idb[off:off + len(flash_boot)] = flash_boot

    code_len = (data_sec + boot_sec) * SECTOR
    struct.pack_into("<I", sec2, 498, crc32_rk(bytes(idb[4 * SECTOR:4 * SECTOR + code_len])))
    idb[2 * SECTOR:3 * SECTOR] = sec2

    for i in (0, 2, 3):
        idb[i * SECTOR:(i + 1) * SECTOR] = rc4(bytes(idb[i * SECTOR:(i + 1) * SECTOR]))
    return bytes(idb)


RKNS_REC0 = 0x78               # first image record in the RKNS FlashHead
RKNS_RECSZ = 88


def validate_rkns(idb, label="ID block"):
    """Check a new-style (RKNS) ID block against its own header.

    The FlashHead sector describes each packed blob as
        +0x00 u16 offset in sectors (from the start of the ID block)
        +0x02 u16 size in sectors
        +0x04 u32 load address (0xFFFFFFFF = none)
        +0x0C u32 kind (1 = DDR init, 2 = loader)
        +0x18 32  SHA-256 of the blob as it sits on the card
    so a card can be checked without any reference image.
    """
    if idb[:4] != b"RKNS":
        log(f"  {label}: magic is {idb[:4]!r}, expected b'RKNS' -- the BootROM "
            f"will not accept this")
        return False
    count = struct.unpack_from("<H", idb, 0x0A)[0]
    if not 1 <= count <= 8:
        log(f"  {label}: implausible image count {count}")
        return False
    ok = True
    log(f"  {label}: RKNS header, {count} images")
    for i in range(count):
        base = RKNS_REC0 + i * RKNS_RECSZ
        off, size = struct.unpack_from("<HH", idb, base)
        load, = struct.unpack_from("<I", idb, base + 4)
        kind, = struct.unpack_from("<I", idb, base + 12)
        want = idb[base + 24:base + 56]
        blob = idb[off * SECTOR:(off + size) * SECTOR]
        kindname = {1: "DDR init", 2: "loader"}.get(kind, f"kind {kind}")
        if len(blob) != size * SECTOR:
            log(f"    image {i} ({kindname}): TRUNCATED -- header wants "
                f"{size} sectors at {off}, only {len(blob) // SECTOR} present")
            ok = False
            continue
        good = hashlib.sha256(blob).digest() == want
        ok &= good
        addr = "-" if load == 0xFFFFFFFF else f"0x{load:08X}"
        log(f"    image {i} ({kindname}): sectors {off}..{off + size - 1}"
            f"  load {addr}  sha256 {'OK' if good else 'MISMATCH'}")
    log(f"  {label}: {'valid' if ok else 'INVALID -- the BootROM will reject it'}")
    return ok


RKFW_TAG = 0x57464B52          # 'RKFW'
_RKFW = "<IHII HBBBBB I IIII"


def extract_update_loader(path):
    """Pull the bootloader (MiniLoaderAll) out of a packed RKFW update.img."""
    with open(path, "rb") as fh:
        head = fh.read(struct.calcsize(_RKFW))
        if len(head) < struct.calcsize(_RKFW):
            return None
        vals = struct.unpack(_RKFW, head)
        if vals[0] != RKFW_TAG or vals[1] != 102:
            return None
        boot_off, boot_size = vals[11], vals[12]
        if not boot_size:
            return None
        fh.seek(boot_off)
        return fh.read(boot_size)


def describe_loader(loader_path):
    """Print everything we can tell about a MiniLoaderAll.bin / loader file."""
    blob = open(loader_path, "rb").read()
    head, entries = parse_rkboot(blob)
    tag = struct.pack("<I", head["tag"]).decode("latin-1")
    chip = struct.pack("<I", head["chip"])
    log(f"  file       : {os.path.basename(loader_path)}  ({human(len(blob))})")
    log(f"  tag        : {tag!r}   header size {head['size']}")
    log(f"  version    : {head['version'] >> 8 & 0xFF}.{head['version'] & 0xFF}"
        f"   merge 0x{head['merge']:08X}")
    log(f"  built      : {head['year']}-{head['month']:02d}-{head['day']:02d} "
        f"{head['hour']:02d}:{head['minute']:02d}:{head['second']:02d}")
    log(f"  chip type  : 0x{head['chip']:08X}  ({chip[::-1].decode('latin-1')!r})")
    log(f"  rc4 flag   : {head['rc4']}      sign flag: {head['sign']}")
    for etype, label in ((1, "code471"), (2, "code472"), (4, "loader ")):
        for name, data in entries.get(etype, {}).items():
            log(f"  {label} entry: {name:<24} {len(data):>8} bytes")
    if head["sign"]:
        log("  !! this loader is SIGNED -- a signed board may reject an "
            "ID block rebuilt by this script")
    kind = "new (RKNS)" if "FlashHead" in entries.get(4, {}) else "legacy (RK28)"
    log(f"  ID block   : would be built in {kind} format")
    return head, entries


def make_idblock(loader_path):
    """Build the ID block image that belongs at LBA 64."""
    blob = open(loader_path, "rb").read()
    head, entries = parse_rkboot(blob)
    loader = entries.get(4, {})                    # ENTRY_LOADER
    for needed in ("FlashData", "FlashBoot"):
        if needed not in loader:
            raise Fail(f"{os.path.basename(loader_path)} has no {needed} entry")

    rc4_off = bool(head["rc4"])
    if "FlashHead" in loader:
        # New-style ID block (RK3506, RK3568, RK3588 ...): "RKNS" header,
        # simple concatenation, each part padded to a 2 KiB boundary.
        parts = [loader["FlashHead"], loader["FlashData"], loader["FlashBoot"]]
        if rc4_off:
            parts = [rc4_per_sector(p) for p in parts]
        secs = [align(len(p), 2048) // SECTOR for p in parts]
        idb = bytearray(sum(secs) * SECTOR)
        at = 0
        for part, nsec in zip(parts, secs):
            idb[at * SECTOR:at * SECTOR + len(part)] = part
            at += nsec
        kind = "new (RKNS)"
        idb = bytes(idb)
    else:
        idb = make_legacy_idb(loader["FlashData"], loader["FlashBoot"], rc4_off)
        kind = "legacy"

    magic = idb[:4]
    note = "RKNS" if magic == b"RKNS" else f"0x{struct.unpack('<I', magic)[0]:08X}"
    log(f"  ID block: {kind} format, {len(idb) // SECTOR} sectors "
        f"({human(len(idb))}), magic {note}")
    if magic == b"RKNS":
        validate_rkns(idb, "  built ID block")
    return idb


# ------------------------------------------------------- Android sparse images

def sparse_header(fh):
    fh.seek(0)
    head = fh.read(28)
    if len(head) < 28 or struct.unpack_from("<I", head, 0)[0] != SPARSE_MAGIC:
        return None
    (_magic, major, _minor, file_hdr_sz, chunk_hdr_sz,
     blk_sz, total_blks, total_chunks, _crc) = struct.unpack("<IHHHHIIII", head)
    if major != 1:
        raise Fail("unsupported Android sparse image version")
    return dict(file_hdr_sz=file_hdr_sz, chunk_hdr_sz=chunk_hdr_sz,
                blk_sz=blk_sz, total_blks=total_blks, total_chunks=total_chunks)


def sparse_chunks(fh, hdr):
    """Yield (byte_offset, data) for a sparse image, skipping DONT_CARE holes."""
    fh.seek(hdr["file_hdr_sz"])
    pos = 0
    for _ in range(hdr["total_chunks"]):
        ch = fh.read(hdr["chunk_hdr_sz"])
        ctype, _r, blocks, total_sz = struct.unpack_from("<HHII", ch, 0)
        payload = total_sz - hdr["chunk_hdr_sz"]
        span = blocks * hdr["blk_sz"]
        if ctype == 0xCAC1:                     # raw
            left = payload
            at = pos
            while left:
                buf = fh.read(min(left, 8 << 20))
                if not buf:
                    raise Fail("truncated sparse image")
                yield at, buf
                at += len(buf)
                left -= len(buf)
        elif ctype == 0xCAC2:                   # fill
            fill = fh.read(payload)
            yield pos, (fill * (span // 4))[:span]
        elif ctype == 0xCAC3:                   # don't care
            pass
        elif ctype == 0xCAC4:                   # crc32
            fh.read(payload)
        else:
            raise Fail(f"unknown sparse chunk type 0x{ctype:04X}")
        pos += span


# --------------------------------------------------------------- GPT writing

def build_gpt(partitions, disk_sectors, disk_guid=None):
    """Byte-compatible with rkdeveloptool's create_gpt_buffer()."""
    master = bytearray(34 * SECTOR)

    # LBA 0: protective MBR
    master[0x1BE + 4] = 0xEE                                   # EFI GPT
    struct.pack_into("<I", master, 0x1BE + 8, 1)               # start sector
    struct.pack_into("<I", master, 0x1BE + 12, 0xFFFFFFFF)     # nr sectors
    struct.pack_into("<H", master, 510, 0xAA55)

    # LBA 2..33: entries
    entries = bytearray(32 * SECTOR)
    for i, part in enumerate(partitions):
        if i >= GPT_ENTRIES:
            raise Fail("more partitions than the GPT can hold")
        base = i * GPT_ENTRY_SIZE
        entries[base:base + 16] = os.urandom(16)               # type GUID
        entries[base + 16:base + 32] = part.uuid or os.urandom(16)
        end = disk_sectors - 34 if part.grow else part.offset + part.size - 1
        struct.pack_into("<QQQ", entries, base + 32, part.offset, end,
                         PART_PROPERTY_BOOTABLE if "bootable" in part.flags else 0)
        name = part.name.encode("utf-16-le")[:71]
        entries[base + 56:base + 56 + len(name)] = name
        part.end_lba = end

    # LBA 1: header
    hdr = bytearray(SECTOR)
    struct.pack_into("<8sIII4xQQQQ16sQIII", hdr, 0,
                     b"EFI PART", 0x00010000, 92, 0,
                     1, disk_sectors - 1, 34, disk_sectors - 34,
                     disk_guid or os.urandom(16),
                     2, GPT_ENTRIES, GPT_ENTRY_SIZE,
                     crc32_le(bytes(entries)))
    struct.pack_into("<I", hdr, 16, crc32_le(bytes(hdr[:92])))

    master[SECTOR:2 * SECTOR] = hdr
    master[2 * SECTOR:34 * SECTOR] = entries

    # backup: 32 sectors of entries then the alternate header
    backup = bytearray(entries) + bytearray(SECTOR)
    bhdr = bytearray(hdr)
    struct.pack_into("<Q", bhdr, 24, disk_sectors - 1)        # my_lba
    struct.pack_into("<Q", bhdr, 32, 1)                       # alternate_lba
    struct.pack_into("<Q", bhdr, 72, disk_sectors - 33)       # entries lba
    struct.pack_into("<I", bhdr, 16, 0)
    struct.pack_into("<I", bhdr, 16, crc32_le(bytes(bhdr[:92])))
    backup[32 * SECTOR:33 * SECTOR] = bhdr

    return bytes(master), bytes(backup)


# ------------------------------------------------------------- device helpers

def device_size(fh):
    BLKGETSIZE64 = 0x80081272
    buf = ctypes.c_ulonglong(0)
    try:
        fcntl.ioctl(fh.fileno(), BLKGETSIZE64, buf)
        return buf.value
    except OSError:
        return os.fstat(fh.fileno()).st_size


def describe_device(path):
    real = os.path.realpath(path)
    name = os.path.basename(real)
    info = {"name": name, "removable": None, "model": "", "mounted": []}
    sysfs = f"/sys/class/block/{name}"
    if os.path.exists(sysfs):
        base = os.path.realpath(sysfs)
        if os.path.exists(f"{base}/../partition"):    # it's a partition
            base = os.path.dirname(base)
        for key, fname in (("removable", "removable"), ("model", "device/model")):
            try:
                with open(f"{base}/{fname}") as fh:
                    val = fh.read().strip()
                info[key] = (val == "1") if key == "removable" else val
            except OSError:
                pass
    try:
        with open("/proc/mounts") as fh:
            for line in fh:
                dev = line.split()[0]
                if dev.startswith(real):
                    info["mounted"].append(line.split()[1])
    except OSError:
        pass
    return info


def check_device(path, force):
    if not os.path.exists(path):
        raise Fail(f"{path} does not exist")
    mode = os.stat(path).st_mode
    if not stat.S_ISBLK(mode):
        if not force:
            raise Fail(f"{path} is not a block device (use --force to write a file)")
    name = os.path.basename(os.path.realpath(path))
    is_partition = os.path.exists(f"/sys/class/block/{name}/partition")
    if not os.path.exists(f"/sys/class/block/{name}"):
        # no sysfs entry to consult -- fall back to the obvious naming rules
        is_partition = bool(re.match(r"(sd[a-z]+|hd[a-z]+)\d+$", name) or
                            re.match(r"(mmcblk\d+|nvme\d+n\d+|loop\d+)p\d+$", name))
    if is_partition and stat.S_ISBLK(mode):
        raise Fail(f"{path} is a partition. Pass the whole card, "
                   f"e.g. /dev/sdb or /dev/mmcblk0")

    info = describe_device(path)
    if info["mounted"]:
        raise Fail(f"{path} has mounted filesystems: {', '.join(info['mounted'])}. "
                   f"Unmount them first (sudo umount {path}*)")
    if info["removable"] is False and not force:
        raise Fail(f"{path} is not a removable device ({info['model'] or 'unknown model'}). "
                   f"Refusing to touch it. Use --force if you are certain.")
    return info


def reread_partition_table(path):
    for cmd in (["partprobe", path], ["blockdev", "--rereadpt", path]):
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            return True
        except (OSError, subprocess.CalledProcessError):
            continue
    return False


# ------------------------------------------------------------------ the work

def drop_cache(fh):
    """Make read-back actually touch the card instead of the page cache."""
    BLKFLSBUF = 0x1261
    try:
        fcntl.ioctl(fh.fileno(), BLKFLSBUF)
    except OSError:
        pass
    try:
        os.posix_fadvise(fh.fileno(), 0, 0, os.POSIX_FADV_DONTNEED)
    except (OSError, AttributeError):
        pass


def write_at(fh, sector, data, label, verify=True):
    fh.seek(sector * SECTOR)
    fh.write(data)
    fh.flush()
    os.fsync(fh.fileno())
    if not verify:
        return
    drop_cache(fh)
    fh.seek(sector * SECTOR)
    back = fh.read(len(data))
    if back != data:
        raise Fail(f"verify failed for {label} at sector {sector}")


def stream_image(fh, part, path, verify):
    """Write <name>.img into its partition, expanding Android-sparse if needed."""
    src = open(path, "rb")
    hdr = sparse_header(src)
    limit = None if part.grow else part.size * SECTOR
    base = part.offset * SECTOR
    digest_src = hashlib.sha256()
    written = 0

    if hdr:
        log(f"    (Android sparse image -> expanding {human(hdr['blk_sz'] * hdr['total_blks'])})")
        span = 0
        for off, chunk in sparse_chunks(src, hdr):
            end = off + len(chunk)
            if limit is not None and end > limit:
                raise Fail(f"{os.path.basename(path)} expands past the '{part.name}' partition")
            fh.seek(base + off)
            fh.write(chunk)
            written += len(chunk)
            span = max(span, end)
        fh.flush()
        os.fsync(fh.fileno())
        if verify:
            drop_cache(fh)
            for off, chunk in sparse_chunks(src, hdr):
                fh.seek(base + off)
                if fh.read(len(chunk)) != chunk:
                    raise Fail(f"verify failed for {part.name} at byte {off}")
        src.close()
        return written
    else:
        size = os.path.getsize(path)
        if limit is not None and size > limit:
            raise Fail(f"{os.path.basename(path)} is {human(size)} but the "
                       f"'{part.name}' partition is only {human(limit)}")
        src.seek(0)
        fh.seek(base)
        while True:
            chunk = src.read(8 << 20)
            if not chunk:
                break
            digest_src.update(chunk)
            fh.write(chunk)
            written += len(chunk)
        payload_size = size
    src.close()
    fh.flush()
    os.fsync(fh.fileno())

    if verify:
        drop_cache(fh)
        fh.seek(base)
        digest_dst = hashlib.sha256()
        left = payload_size
        while left:
            chunk = fh.read(min(left, 8 << 20))
            if not chunk:
                raise Fail("short read while verifying")
            digest_dst.update(chunk)
            left -= len(chunk)
        if digest_dst.digest() != digest_src.digest():
            raise Fail(f"verify failed for {part.name} ({os.path.basename(path)})")
    return payload_size


def find_image(directory, *names):
    listing = {f.lower(): f for f in os.listdir(directory)}
    for want in names:
        hit = listing.get(want.lower())
        if hit:
            return os.path.join(directory, hit)
    return None


def main():
    ap = argparse.ArgumentParser(
        description="Write a Rockchip/Luckfox MicroSD image bundle to an SD card.",
        epilog="example: sudo %(prog)s Ubuntu_Luckfox_Lyra_MicroSD_250417 /dev/sdb")
    ap.add_argument("image_dir", help="folder holding parameter.txt and the .img files")
    ap.add_argument("device", nargs="?",
                    help="whole SD card device, e.g. /dev/sdb or /dev/mmcblk0")
    ap.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")
    ap.add_argument("-n", "--dry-run", action="store_true", help="show the plan, write nothing")
    ap.add_argument("--force", action="store_true",
                    help="allow non-removable devices and plain files")
    ap.add_argument("--no-verify", action="store_true", help="skip read-back verification")
    ap.add_argument("--skip-loader", action="store_true",
                    help="leave the boot loader area (LBA 64..) alone")
    ap.add_argument("--skip-gpt", action="store_true", help="do not rewrite the partition table")
    ap.add_argument("--only", metavar="NAME", action="append",
                    help="write only these partitions (repeatable)")
    ap.add_argument("--inspect", action="store_true",
                    help="describe the bundle and the loader, write nothing "
                         "(no device argument needed)")
    ap.add_argument("--dump-idblock", metavar="FILE",
                    help="save the ID block this script would write, for comparison")
    ap.add_argument("--loader-only", action="store_true",
                    help="write just the ID block at LBA 64; leave the GPT and "
                         "every partition untouched")
    ap.add_argument("--check", action="store_true",
                    help="read the ID block back off the card and validate it "
                         "against its own RKNS header, then exit")
    ap.add_argument("--loader-from-update", action="store_true",
                    help="build the ID block from the loader packed inside "
                         "update.img instead of MiniLoaderAll.bin")
    args = ap.parse_args()
    if args.loader_only:
        args.skip_gpt = True
        args.only = ["\x00none"]          # matches no partition name
    if not args.device and not (args.inspect or args.dump_idblock):
        ap.error("a device is required (or use --inspect / --dump-idblock)")

    if args.check:
        if not args.device:
            ap.error("--check needs a device or image file")
        with open(args.device, "rb") as dev:
            # accept either a whole card (ID block at LBA 64) or a bare dump
            # of the ID block itself (e.g. `upgrade_tool rl 64 384 dump.img`)
            start = IDB_LBA
            if dev.read(4) == b"RKNS":
                start = 0
            dev.seek(start * SECTOR)
            head = dev.read(4 * SECTOR)
            if head[:4] != b"RKNS":
                log(f"sector {IDB_LBA} of {args.device} starts with {head[:4]!r}, "
                    f"not b'RKNS' -- no usable ID block is present")
                return 1
            count = struct.unpack_from("<H", head, 0x0A)[0]
            total = 0
            for i in range(min(count, 8)):
                off, size = struct.unpack_from("<HH", head, RKNS_REC0 + i * RKNS_RECSZ)
                total = max(total, off + size)
            dev.seek(start * SECTOR)
            idb = dev.read(total * SECTOR)
        where = f"sector {start}" if start else "offset 0 (bare ID block dump)"
        log(f"{args.device}: ID block at {where}, {total} sectors")
        return 0 if validate_rkns(idb, "  on-card ID block") else 1

    directory = args.image_dir.rstrip("/")
    if not os.path.isdir(directory):
        raise Fail(f"{directory} is not a directory")

    param_path = find_image(directory, "parameter.txt")
    if not param_path:
        raise Fail(f"no parameter.txt in {directory}")

    partitions, meta = parse_parameter(param_path)

    log(f"Bundle:  {directory}")
    if meta.get("MACHINE_MODEL"):
        log(f"Machine: {meta.get('MACHINE_MODEL')}  firmware {meta.get('FIRMWARE_VER', '?')}"
            f"  table {meta.get('TYPE', '?')}")

    # attach an image file to each partition
    for part in partitions:
        part.image = find_image(directory, f"{part.name}.img")
        if part.uuid is None and part.name == "rootfs":
            log("  note: no uuid: line for 'rootfs' in parameter.txt -- a random "
                "PARTUUID will be used")

    loader_path = None
    prebuilt_idb = find_image(directory, "idblock.img")
    if not args.skip_loader:
        loader_path = prebuilt_idb or find_image(
            directory, "MiniLoaderAll.bin", "MiniLoader.bin", "loader.bin")
        if not loader_path:
            raise Fail("found neither idblock.img nor MiniLoaderAll.bin; "
                       "pass --skip-loader to keep the card's existing loader")

    log()
    log("Partition table from parameter.txt:")
    log(f"  {'name':<10} {'start':>10} {'sectors':>10} {'size':>10}  image")
    for part in partitions:
        size = "grow" if part.grow else str(part.size)
        pretty = "-" if part.grow else human(part.size * SECTOR)
        img = os.path.basename(part.image) if part.image else "(none)"
        log(f"  {part.name:<10} {part.offset:>10} {size:>10} {pretty:>10}  {img}")
    if part_uuid := next((p.uuid for p in partitions if p.name == "rootfs"), None):
        log(f"  rootfs PARTUUID: {gpt_bytes_to_uuid(part_uuid)}")

    log()
    idb = None
    if loader_path:
        log(f"Loader:  {os.path.basename(loader_path)}")
        if args.loader_from_update:
            update = find_image(directory, "update.img")
            if not update:
                raise Fail("--loader-from-update given but there is no update.img")
            embedded = extract_update_loader(update)
            if embedded is None:
                raise Fail(f"{os.path.basename(update)} is not a packed RKFW image")
            tmp = tempfile.NamedTemporaryFile(prefix="rk-loader-", suffix=".bin",
                                              delete=False)
            tmp.write(embedded)
            tmp.close()
            loader_path = tmp.name
            prebuilt_idb = None
            log(f"  using the loader packed inside update.img ({human(len(embedded))})")
            idb = make_idblock(loader_path)
        elif prebuilt_idb:
            idb = open(loader_path, "rb").read()
            log(f"  ID block: taken as-is, {len(idb) // SECTOR} sectors")
        else:
            idb = make_idblock(loader_path)
        room = partitions[0].offset - IDB_LBA
        if len(idb) // SECTOR > room:
            raise Fail(f"ID block needs {len(idb) // SECTOR} sectors but only {room} "
                       f"are free before the first partition")
    else:
        log("Loader:  skipped (--skip-loader)")

    if loader_path and not prebuilt_idb:
        log()
        log("Loader details:")
        describe_loader(loader_path)

        update = find_image(directory, "update.img")
        if update:
            embedded = extract_update_loader(update)
            if embedded is None:
                log("  update.img: not a packed RKFW image (or no bootloader inside)")
            else:
                same = embedded == open(loader_path, "rb").read()
                log(f"  update.img: carries a {human(len(embedded))} loader -- "
                    f"{'identical to' if same else 'DIFFERENT from'} "
                    f"{os.path.basename(loader_path)}")
                if not same:
                    log("  !! the standalone loader and the one inside update.img "
                        "disagree; the board expects the one from update.img")

    if args.dump_idblock and idb is not None:
        with open(args.dump_idblock, "wb") as fh:
            fh.write(idb)
        log(f"\n  ID block written to {args.dump_idblock} "
            f"({len(idb)} bytes, {len(idb) // SECTOR} sectors)")

    if args.inspect or (args.dump_idblock and not args.device):
        log("\nInspect only -- nothing written to any device.")
        return 0

    extra = [f for f in ("update.img",) if find_image(directory, f)]
    if extra:
        log(f"  ignoring {', '.join(extra)} (packed RKFW firmware, not a raw image)")

    # --- open the target -----------------------------------------------------
    info = check_device(args.device, args.force)
    mode = "rb" if args.dry_run else "rb+"
    with open(args.device, mode) as dev:
        total_bytes = device_size(dev)
        total_sectors = total_bytes // SECTOR
        log()
        log(f"Target:  {args.device}  {human(total_bytes)}  "
            f"({total_sectors} sectors)"
            f"{'  [removable]' if info['removable'] else ''}"
            f"{'  ' + info['model'] if info['model'] else ''}")

        last = max(p.offset + (0 if p.grow else p.size) for p in partitions)
        if total_sectors < last + 34:
            raise Fail(f"card is too small: needs at least {human((last + 34) * SECTOR)}")

        if not args.yes and not args.dry_run:
            log()
            log(f"!! Everything on {args.device} will be destroyed.")
            if input("Type YES to continue: ").strip() != "YES":
                raise Fail("aborted")

        if args.dry_run:
            log("\nDry run -- nothing written.")
            return 0

        log()
        if not args.skip_gpt:
            master, backup = build_gpt(partitions, total_sectors)
            write_at(dev, 0, master, "primary GPT", not args.no_verify)
            write_at(dev, total_sectors - 33, backup, "backup GPT", not args.no_verify)
            log(f"  wrote GPT ({len(partitions)} partitions, backup at sector "
                f"{total_sectors - 33})")
        else:
            log("  GPT: skipped")

        if idb is not None:
            write_at(dev, IDB_LBA, idb, "ID block", not args.no_verify)
            log(f"  wrote ID block at sector {IDB_LBA} ({human(len(idb))})")

        for part in partitions:
            if args.only and part.name not in args.only:
                continue
            if not part.image:
                log(f"  {part.name}: no image in the bundle, skipped")
                continue
            log(f"  writing {part.name} <- {os.path.basename(part.image)} "
                f"at sector {part.offset} ...")
            n = stream_image(dev, part, part.image, not args.no_verify)
            log(f"    {human(n)} written{'' if args.no_verify else ' and verified'}")

        os.fsync(dev.fileno())

    subprocess.run(["sync"], check=False)
    if stat.S_ISBLK(os.stat(args.device).st_mode):
        reread_partition_table(args.device)

    log()
    log("Done. Card layout:")
    for part in partitions:
        end = getattr(part, "end_lba", None)
        log(f"  {part.name:<10} sectors {part.offset}..{end if end else '?'}")
    log()
    log("Insert it in the board and power on. If the rootfs does not fill the card,")
    log("run  resize2fs /dev/mmcblk*p3  once you have booted.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(130)
    except BrokenPipeError:
        try:
            sys.stdout.close()
        finally:
            os._exit(0)
