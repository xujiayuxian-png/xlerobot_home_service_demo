import asyncio
from types import SimpleNamespace, MethodType
from unittest.mock import patch
import pytest

from xlerobot_calibration_tools.hover_node import HoverNode


def fake(tmp_path, fail=None):
    calls = []
    node = SimpleNamespace(operation_lock=asyncio.Lock(), cancel_requested=False, root=tmp_path)
    async def prepare(**kwargs):
        calls.append('ready' if kwargs.get('return_ready') else 'prepare')
        node.check_cancel()
    async def preview(target):
        calls.append(target)
        node.plan = {'id': target}
        if fail == target: raise ValueError('no safe plan')
    async def execute(plan): calls.append('execute:' + plan)
    async def measure():
        calls.append('measure')
        node.result = {'target': node.plan['id']}
        if fail == 'cancel': node.cancel_requested = True
    node.prepare_observation, node.preview, node.execute, node.measure = prepare, preview, execute, measure
    node.check_cancel = MethodType(HoverNode.check_cancel, node)
    node.save_suite = lambda run, rows, state, message: calls.append(('saved', len(rows), state))
    return node, calls


@pytest.mark.parametrize('fail', [None, 'right', 'cancel'])
def test_auto_uses_existing_steps_stops_on_failure_and_does_not_return_after_cancel(tmp_path, fail):
    async def check():
        node, calls = fake(tmp_path, fail)
        with patch('xlerobot_calibration_tools.hover_node.summarize_arrival', lambda r: r):
            await HoverNode.automatic(node)
        if fail is None:
            assert [c for c in calls if isinstance(c, str)] == [
                'prepare', 'center', 'execute:center', 'measure', 'right', 'execute:right', 'measure',
                'left', 'execute:left', 'measure', 'prepare', 'ready']
            assert calls[-1] == ('saved', 3, 'COMPLETED')
        else:
            assert 'ready' not in calls and 'execute:right' not in calls
            assert calls[-1] == ('saved', 1, 'INTERRUPTED')
    asyncio.run(check())
