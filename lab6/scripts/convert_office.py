"""Convert only required Office topics, retaining original message timestamps.

For a full ZIP, extract the DB and run without --prefix. Prefix mode streams a
contiguous ZIP prefix for early validation and stops at the first missing row.
"""
import argparse
from collections import Counter
from pathlib import Path
import sqlite3
import struct
import zlib
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

TOPICS = {'/tesse/odom', '/tesse/left_cam/rgb/image_raw',
          '/tesse/left_cam/camera_info', '/tesse/depth_cam/mono/image_raw',
          '/tesse/depth_cam/camera_info'}

def extract_prefix(source, db):
    limit = source.stat().st_size
    with source.open('rb') as f, db.open('wb') as out:
        for _ in range(3):
            sig, _, flag, method, _, _, _, _, _, n, x = struct.unpack('<I5H3I2H', f.read(30))
            name = f.read(n).decode()
            f.read(x)
            if sig != 0x04034b50:
                raise ValueError('Invalid ZIP entry')
            if name.endswith('/'):
                continue
            if method != 8:
                raise ValueError('Expected deflated Office entry')
            dec = zlib.decompressobj(-15)
            if name.endswith('yaml'):
                dec.decompress(f.read(8192))
                if not dec.eof or not flag & 8:
                    raise ValueError('Unexpected metadata descriptor')
                f.seek(-len(dec.unused_data)+16, 1)
                continue
            if not name.endswith('.db3'):
                raise ValueError(name)
            while f.tell() < limit:
                out.write(dec.decompress(f.read(min(1024*1024, limit-f.tell()))))
            break
    print('Received ZIP prefix bytes:', limit)
    print('Complete decompressed prefix bytes:', db.stat().st_size)
    with db.open('r+b') as f:
        header = f.read(100)
        page = struct.unpack_from('>H', header, 16)[0]
        pages = struct.unpack_from('>I', header, 28)[0]
        # Missing pages are sparse zeroes. SQLite raises when it first reaches
        # an absent page. Only rows it fully parses are serialized below.
        f.truncate(page*pages)

def convert(db, out, prefix):
    store = get_typestore(Stores.ROS1_NOETIC)
    cdr_store = get_typestore(Stores.ROS2_HUMBLE)
    counts = Counter()
    with sqlite3.connect(f'file:{db.resolve()}?mode=ro', uri=True) as conn, Writer(out) as bag:
        topics = {key: (name, typ) for key,name,typ in conn.execute('SELECT id,name,type FROM topics')
                  if name in TOPICS}
        if {v[0] for v in topics.values()} != TOPICS:
            raise ValueError('Office topics missing')
        handles = {key: bag.add_connection(name,typ,typestore=store)
                   for key,(name,typ) in topics.items()}
        rows = conn.execute('SELECT topic_id,timestamp,data FROM messages NOT INDEXED')
        while True:
            try:
                row = rows.fetchone()
            except sqlite3.DatabaseError:
                if not prefix:
                    raise
                print('Stopped at first missing SQLite page in the archive prefix')
                break
            if row is None:
                break
            key, stamp, cdr = row
            if key not in handles:
                continue
            bag.write(handles[key], stamp, cdr_store.cdr_to_ros1(cdr, topics[key][1]))
            counts[topics[key][0]] += 1
    print(dict(counts))
    if any(not counts[topic] for topic in TOPICS):
        raise RuntimeError('Missing required message data')

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('source', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument('--prefix', type=Path, help='temporary SQLite destination for prefix mode')
    ap.add_argument("--partial", action="store_true", help="read an already extracted sparse prefix")
    args = ap.parse_args()
    if args.prefix:
        extract_prefix(args.source,args.prefix)
    convert(args.prefix or args.source,args.output,bool(args.prefix) or args.partial)
