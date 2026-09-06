import asyncio
import threading
from types import SimpleNamespace

from aiohttp import web
import pytest
from std_srvs.srv import Trigger

from xlerobot_hmi.operator_console import OperatorConsoleNode


@pytest.mark.parametrize('operation,success', [('release', True), ('reset', True), ('reset', False)])
def test_recovery_only_clears_display_after_confirmed_reset(operation, success):
    async def exercise():
        events = []
        node = object.__new__(OperatorConsoleNode)
        node._collection_lock = threading.Lock()
        original = {'dataset_id': 'trial', 'episode_id': 'one', 'status': 'FAILED'}
        node.active_collection = dict(original)
        node.collection_goal_handle = None
        node.events = SimpleNamespace(publish=lambda *args: events.append(args))
        def call(request):
            assert isinstance(request, Trigger.Request)
            future = asyncio.get_running_loop().create_future()
            future.set_result(Trigger.Response(success=success, message='result'))
            return future
        client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
        node.collection_release_client = node.collection_reset_client = client
        if success:
            await node.recover_collection(operation)
        else:
            with pytest.raises(web.HTTPConflict):
                await node.recover_collection(operation)
        assert node.active_collection == (None if operation == 'reset' and success else original)
        assert events == ([('collection', None)] if operation == 'reset' and success else [])
        assert not node._collection_operation
    asyncio.run(exercise())


def test_release_drains_active_episode_before_torque_service():
    async def exercise():
        events = []
        node = object.__new__(OperatorConsoleNode)
        node._collection_lock = threading.Lock()
        node.active_collection = {'dataset_id': 'trial', 'episode_id': 'one', 'status': 'RUNNING'}
        async def cancel(dataset, episode):
            assert (dataset, episode) == ('trial', 'one')
            events.append('cancel and drain')
            node.active_collection['status'] = 'CANCELED'
        node.cancel_collection = cancel
        def call(request):
            events.append('release torque')
            future = asyncio.get_running_loop().create_future()
            future.set_result(Trigger.Response(success=True))
            return future
        node.collection_release_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
        await node.recover_collection('release')
        assert events == ['cancel and drain', 'release torque']
    asyncio.run(exercise())
