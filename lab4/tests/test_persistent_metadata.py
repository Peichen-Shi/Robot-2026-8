"""Exercise actual callback sockets, fragmented replies and reconnects."""
import importlib.util,socket,struct,sys,threading,unittest
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
