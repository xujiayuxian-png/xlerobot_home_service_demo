"""Submit one typed fetch-and-deliver task and stream capability feedback."""

from __future__ import annotations

import argparse
import sys

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xlerobot_interfaces.action import ExecuteTask
from xlerobot_interfaces.msg import CapabilityError


def argument_parser():
    """Define the deliberately small task-level CLI."""
    parser = argparse.ArgumentParser(
        description='Submit one fetch-and-deliver task through ExecuteTask.'
    )
    parser.add_argument('object_id', help='concrete object name or identifier')
    parser.add_argument('--source', default='table', help='named source place')
    parser.add_argument(
        '--recipient', default='nearest_person', help='recipient identifier'
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='request non-dry-run task execution; use only in an authorized run',
    )
    parser.add_argument('--timeout', type=float, default=360.0, help='result timeout in seconds')
    return parser


def build_goal(arguments):
    """Map CLI terms to the stable task contract."""
    fields = {
        'object_id': arguments.object_id.strip(),
        'source': arguments.source.strip(),
        'recipient': arguments.recipient.strip(),
    }
    empty = [name for name, value in fields.items() if not value]
    if empty:
        raise ValueError(f'task fields must not be empty: {empty}')
    if arguments.timeout <= 0.0:
        raise ValueError('timeout must be positive')
    goal = ExecuteTask.Goal()
    goal.object_id = fields['object_id']
    goal.source_place = fields['source']
    goal.recipient_id = fields['recipient']
    goal.dry_run = not arguments.execute
    return goal


class FetchDeliverClient(Node):
    """One-shot action client; the HMI does not reproduce task semantics."""

    def __init__(self):
        super().__init__('fetch_deliver_cli')
        self.client = ActionClient(self, ExecuteTask, '/execute_task')

    def submit(self, goal, timeout_s):
        """Return a wrapped action result, canceling on the caller's timeout."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('ExecuteTask action is unavailable')
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            raise TimeoutError('timed out waiting for task goal response')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('ExecuteTask goal was rejected')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise TimeoutError('task timed out; cancellation was requested')
        return result_future.result()

    @staticmethod
    def _feedback(message):
        feedback = message.feedback
        state = feedback.state
        print(
            f'[{state.progress:5.1%}] {feedback.current_capability}: '
            f'{state.phase} {state.message}'.rstrip(),
            flush=True,
        )


def run(argv=None):
    """Run the CLI and return a process-style status code."""
    parser = argument_parser()
    try:
        arguments = parser.parse_args(argv)
        goal = build_goal(arguments)
    except ValueError as exc:
        parser.error(str(exc))
    rclpy.init()
    node = FetchDeliverClient()
    try:
        mode = 'EXECUTE' if arguments.execute else 'DRY-RUN'
        print(
            f'{mode}: object={goal.object_id} source={goal.source_place} '
            f'recipient={goal.recipient_id}',
            flush=True,
        )
        wrapped = node.submit(goal, arguments.timeout)
        result = wrapped.result
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error.code == CapabilityError.NONE
        ):
            print(f'complete: {result.completed_object_id}', flush=True)
            return 0
        print(
            f'failed: status={wrapped.status} code={result.error.code} '
            f'message={result.error.message}',
            file=sys.stderr,
            flush=True,
        )
        return 1
    except (RuntimeError, TimeoutError) as exc:
        print(f'failed: {exc}', file=sys.stderr, flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Console entry point."""
    raise SystemExit(run())


if __name__ == '__main__':
    main()
