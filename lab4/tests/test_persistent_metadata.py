"""Exercise actual callback sockets, fragmented replies and reconnects."""
import importlib.util,socket,struct,sys,threading,unittest
from unittest.mock import patch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'code/tesse-ros-bridge/tesse-interface/src'))
spec=importlib.util.spec_from_file_location('persistent_metadata',ROOT/'code/tesse-ros-bridge/ROS/src/tesse_ros_bridge/persistent_metadata.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class Server:
 def __init__(self):self.port=None;self.broken=False
 def send(self,_):
  def reply():
   with socket.create_connection(('127.0.0.1',self.port),timeout=2) as s:
    payload=b'<metadata>complete</metadata>'
    frame=b'meta'+struct.pack('I',len(payload))+payload
    if self.broken:frame=frame[:12]
    for byte in frame:s.sendall(bytes([byte]))
  threading.Thread(target=reply,daemon=True).start()
class MetadataTest(unittest.TestCase):
 def test_invalid_timeout(self):
  for timeout in (0, -1, float('nan'), float('inf')):
   with self.assertRaises(ValueError):
    module.PersistentMetadataClient(None,'127.0.0.1',0,timeout=timeout)

 def test_fragments_share_one_deadline(self):
  # A deterministic slow peer: each byte needs 30 ms, below the old
  # per-recv 100 ms timeout, but the complete response exceeds its budget.
  class SlowPeer:
   now=0.
   def settimeout(self,value): self.timeout=value
   def recv(self,length):
    if self.timeout < .03: raise socket.timeout('fragment exceeded remaining budget')
    self.now+=.03
    return b'x'
  peer=SlowPeer()
  with patch.object(module.time,'monotonic',side_effect=lambda:peer.now):
   with self.assertRaises(socket.timeout):
    module.PersistentMetadataClient.receive_exact(peer,10,deadline=.1)
  self.assertLessEqual(peer.now,.1)

 def test_expired_deadline_does_not_read(self):
  class UnreadablePeer:
   def recv(self,length): raise AssertionError('must not receive after deadline')
  with patch.object(module.time,'monotonic',return_value=1.):
   with self.assertRaises(socket.timeout):
    module.PersistentMetadataClient.receive_exact(UnreadablePeer(),4,deadline=1.)

 def test_fragmented_replies_and_reconnect(self):
  env=Server();client=module.PersistentMetadataClient(env,'127.0.0.1',0,timeout=2)
  env.port=client.listener.getsockname()[1]
  try:
   for _ in range(3):self.assertEqual(client.request(),b'<metadata>complete</metadata>')
   env.broken=True
   with self.assertRaises(ConnectionError):client.request()
   env.broken=False
   self.assertEqual(client.request(),b'<metadata>complete</metadata>')
  finally:client.close()
if __name__=='__main__':unittest.main()
