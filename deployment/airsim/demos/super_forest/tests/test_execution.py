"""Local checks for explicit cross-machine execution configuration."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

APP=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('execution',APP/'robopi_execution.py')
execution=importlib.util.module_from_spec(spec)
spec.loader.exec_module(execution)


class ExecutionTest(unittest.TestCase):
    def test_dds_binds_local_and_targets_peer(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'dds.xml'
            execution.dds_profile(path,'192.0.2.1','192.0.2.2')
            root=ET.parse(path)
            ns={'f':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            self.assertEqual(root.find('.//f:interfaceWhiteList/f:address',ns).text,'192.0.2.1')
            self.assertEqual(root.find('.//f:initialPeersList/f:locator/f:udpv4/f:address',ns).text,'192.0.2.2')
            self.assertEqual([x.text for x in root.findall('.//f:initialPeersList/f:locator/f:udpv4/f:address',ns)],
                ['192.0.2.2','192.0.2.1','127.0.0.1','239.255.0.1'])
            with self.assertRaises(ValueError):execution.dds_profile(path,'bad','192.0.2.2')

    def test_wall_clock_skew_rejected_before_board_mutation(self):
        class Link:
            def clock_offset(self):return -2.77
            def board(self,*args,**kwargs):raise AssertionError('Board mutation attempted')
        with tempfile.TemporaryDirectory() as folder:
            board=execution.BoardExecution.__new__(execution.BoardExecution)
            board.out=Path(folder);board.link=Link()
            with self.assertRaisesRegex(RuntimeError,'wall clocks differ'):board.prepare()
            self.assertTrue((board.out/'clock-offset.json').exists())

    def test_runtime_skew_is_detected_after_initially_valid_clock(self):
        import json
        state={'pid':123,'ticks':'456','boot_id':'test','alive':True}
        class Link:
            offset=-.1
            def clock_offset(self):return self.offset
            def board(self,*args,**kwargs):return json.dumps(state)
        with tempfile.TemporaryDirectory() as folder:
            board=execution.BoardExecution.__new__(execution.BoardExecution)
            board.out=Path(folder);board.link=Link();board.env=''
            (board.out/'board-start.json').write_text(json.dumps(state))
            board.check()
            board.link.offset=-2.77
            with self.assertRaisesRegex(RuntimeError,'runtime'):board.check()
            self.assertEqual(len((board.out/'clock-offsets.jsonl').read_text().splitlines()),2)

    def test_unconfirmed_recovery_retains_board(self):
        with tempfile.TemporaryDirectory() as folder:
            board=execution.BoardExecution.__new__(execution.BoardExecution)
            board.out=Path(folder)
            board.started=True
            with self.assertRaisesRegex(RuntimeError,'Recovery is unconfirmed'):board.stop()
            self.assertTrue(board.started)


if __name__=='__main__':unittest.main()
