import asyncio
from types import SimpleNamespace

import pytest
from aiohttp import web
from xlerobot_hmi.operator_console import ConsoleApplication


@pytest.mark.parametrize('workflow,endpoint', [('head_camera', 'head'), ('right_handeye', 'handeye')])
@pytest.mark.parametrize('case', ['ok', 'unconfirmed', 'wrong_unit', 'running', 'preview', 'rejected'])
def test_visual_reset_contract(workflow, endpoint, case):
    calls = []
    values = dict(workspace='calibration', enable_engineering_tools=True,
                  calibration_workflow=workflow, unit_id='unit')
    node = SimpleNamespace(parameter=lambda k: values[k],
        head_calibration_snapshot=lambda: dict(state_fresh=True, running=case == 'running'),
        manual_control=SimpleNamespace(action_active=lambda: case == 'preview'),
        head_calibration_reset_client=object(), history=SimpleNamespace(audit=lambda *args: None))
    app = ConsoleApplication(node)
    async def call(*args):
        calls.append(args)
        return SimpleNamespace(success=case != 'rejected', message='archived')
    app._call_service = call
    async def payload():
        return dict(confirm=case != 'unconfirmed', unit_id='other' if case == 'wrong_unit' else 'unit')
    request = SimpleNamespace(path=f'/api/v1/calibrations/{endpoint}/reset', json=payload)
    if case == 'ok':
        assert asyncio.run(app.reset_visual_calibration(request)).status == 200
        assert len(calls) == 1
    else:
        with pytest.raises(web.HTTPException):
            asyncio.run(app.reset_visual_calibration(request))
        assert len(calls) == (1 if case == 'rejected' else 0)
    assert app._head_auto_inflight is False
