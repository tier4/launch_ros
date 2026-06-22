# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the LoadComposableNodes Action."""

import pathlib
import threading
from unittest.mock import MagicMock
from unittest.mock import patch

from composition_interfaces.srv import LoadNode

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.utilities import get_node_name_count

import pytest

from rcl_interfaces.msg import ParameterType

import rclpy
import rclpy.context
import rclpy.executors
import rclpy.node

TEST_CONTAINER_NAME = 'mock_component_container'
TEST_NODE_NAME = 'test_load_composable_nodes_node'


class MockComponentContainer(rclpy.node.Node):

    def __init__(self):
        # List of LoadNode requests received
        self.requests = []

        self._context = rclpy.context.Context()
        rclpy.init(context=self._context)

        super().__init__(TEST_CONTAINER_NAME, context=self._context)

        self.load_node_service = self.create_service(
            LoadNode,
            '~/_container/load_node',
            self.load_node_callback
        )

        self._executor = rclpy.executors.SingleThreadedExecutor(context=self._context)

        # Start spinning in a thread
        self._thread = threading.Thread(
            target=rclpy.spin,
            args=(self, self._executor),
            daemon=True
        )
        self._thread.start()

    def load_node_callback(self, request, response):
        self.requests.append(request)
        response.success = True
        if request.node_namespace == '/':
            response.full_node_name = f'/{request.node_name}'
        else:
            response.full_node_name = f'{request.node_namespace}/{request.node_name}'
        response.unique_id = len(self.requests)
        return response

    def shutdown(self):
        self._executor.shutdown()
        rclpy.shutdown(context=self._context)
        self.destroy_node()
        self._thread.join()


def _assert_launch_no_errors(actions):
    ld = LaunchDescription(actions)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    return ls.context


def _load_composable_node(
    *,
    package,
    plugin,
    name,
    namespace='',
    condition=None,
    parameters=None,
    remappings=None,
    target_container=f'/{TEST_CONTAINER_NAME}'
):
    return LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[
            ComposableNode(
                condition=condition,
                package=package,
                plugin=plugin,
                name=name,
                namespace=namespace,
                parameters=parameters,
                remappings=remappings,
            )
        ])


@pytest.fixture
def mock_component_container():
    container = MockComponentContainer()
    yield container
    container.shutdown()


def test_load_node(mock_component_container):
    """Test loading a node."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace'
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_conditions(mock_component_container):
    """Test loading nodes with conditions scoped to a group."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name_true',
            namespace='test_node_namespace',
            condition=IfCondition('True')
        ),
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name_false',
            namespace='test_node_namespace',
            condition=IfCondition('False')
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name_true') == 1
    assert get_node_name_count(context, '/test_node_namespace/test_node_name_false') == 0

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name_true'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_remaps(mock_component_container):
    """Test loading a node with remappings."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace',
            remappings=[
                ('test_topic1', 'test_remap_topic1'),
                ('test/topic/two', 'test/remap_topic2')
            ]
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 2
    assert request.remap_rules[0] == 'test_topic1:=test_remap_topic1'
    assert request.remap_rules[1] == 'test/topic/two:=test/remap_topic2'
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_params(mock_component_container):
    """Test loading a node with parameters."""
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace',
            parameters=[{
                'test_param1': 'test_value_param1',
                'test.param2': 42.0,
            }]
        )
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 2
    assert request.parameters[0].name == 'test_param1'
    assert request.parameters[0].value.type == ParameterType.PARAMETER_STRING
    assert request.parameters[0].value.string_value == 'test_value_param1'
    assert request.parameters[1].name == 'test.param2'
    assert request.parameters[1].value.type == ParameterType.PARAMETER_DOUBLE
    assert request.parameters[1].value.double_value == 42.0
    assert len(request.extra_arguments) == 0


def test_load_node_with_param_file(mock_component_container):
    """Test loading a node with with parameters specified in yaml files."""
    parameters_file_dir = pathlib.Path(__file__).resolve().parent

    # Case 1: no node name in yaml file
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='test_node_namespace',
            parameters=[
                parameters_file_dir / 'example_parameters_no_name.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.parameters) == 2
    assert request.parameters[0].name == 'some_int'
    assert request.parameters[0].value.integer_value == 42
    assert request.parameters[1].name == 'a_string'
    assert request.parameters[1].value.string_value == 'Hello world'

    # Case 2: node name with namespace
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='ns',
            parameters=[
                parameters_file_dir / 'example_parameters_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/ns/test_node_name') == 1
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/ns'
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'param'
    assert request.parameters[0].value.integer_value == 1

    # Case 3: nested node name with namespace
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='my_node',
            namespace='my_ns',
            parameters=[
                parameters_file_dir / 'example_parameters_nested_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/my_ns/my_node') == 1
    assert request.node_name == 'my_node'
    assert request.node_namespace == '/my_ns'
    assert len(request.parameters) == 5
    assert request.parameters[0].name == 'some_int'
    assert request.parameters[0].value.integer_value == 42
    assert request.parameters[1].name == 'a_string'
    assert request.parameters[1].value.string_value == 'Hello world'
    assert request.parameters[2].value.string_value == ''

    # Case 4: node name without namespace
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='',
            parameters=[
                parameters_file_dir / 'example_parameters_no_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/test_node_name') == 1
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/'
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'param'
    assert request.parameters[0].value.integer_value == 2

    # Case 5: wildcard
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='my_node',
            namespace='wildcard_ns',
            parameters=[
                parameters_file_dir / 'example_parameters_wildcard.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/wildcard_ns/my_node') == 1
    assert request.node_name == 'my_node'
    assert request.node_namespace == '/wildcard_ns'
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'param'
    assert request.parameters[0].value.string_value == 'wildcard'

    # Case 6: multiple entries (params with node name should take precedence over wildcard params)
    context = _assert_launch_no_errors([
        LoadComposableNodes(  # Load in same action so it happens sequentially
            target_container=f'/{TEST_CONTAINER_NAME}',
            composable_node_descriptions=[
                ComposableNode(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='node_1',
                    namespace='ns_1',
                    parameters=[
                        parameters_file_dir / 'example_parameters_multiple_entries.yaml'
                    ]
                ),
                ComposableNode(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='node_2',
                    namespace='ns_2',
                    parameters=[
                        parameters_file_dir / 'example_parameters_multiple_entries.yaml'
                    ]
                )
            ]
        )
    ])
    request = mock_component_container.requests[-2]
    assert get_node_name_count(context, '/ns_1/node_1') == 1
    assert request.node_name == 'node_1'
    assert request.node_namespace == '/ns_1'
    assert len(request.parameters) == 4
    assert request.parameters[0].name == 'param_1'
    assert request.parameters[0].value.integer_value == 1
    assert request.parameters[1].name == 'param_2'
    assert request.parameters[1].value.integer_value == 2
    assert request.parameters[2].name == 'param_3'
    assert request.parameters[2].value.integer_value == 33
    assert request.parameters[3].name == 'param_4'
    assert request.parameters[3].value.integer_value == 4

    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/ns_2/node_2') == 1
    assert request.node_name == 'node_2'
    assert request.node_namespace == '/ns_2'
    assert len(request.parameters) == 2
    assert request.parameters[0].name == 'param_2'
    assert request.parameters[0].value.integer_value == 22
    assert request.parameters[1].name == 'param_3'
    assert request.parameters[1].value.integer_value == 3

    # Case 7: multiple nodes in one namespace
    context = _assert_launch_no_errors([
        LoadComposableNodes(  # Load in same action so it happens sequentially
            target_container=f'/{TEST_CONTAINER_NAME}',
            composable_node_descriptions=[
                ComposableNode(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='node_1',
                    namespace='ns_1',
                    parameters=[
                        parameters_file_dir / 'example_parameters_multiple_nodes.yaml'
                    ]
                ),
                ComposableNode(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='node_2',
                    namespace='ns_1',
                    parameters=[
                        parameters_file_dir / 'example_parameters_multiple_nodes.yaml'
                    ]
                )
            ]
        )
    ])
    request = mock_component_container.requests[-2]
    assert get_node_name_count(context, '/ns_1/node_1') == 1
    assert request.node_name == 'node_1'
    assert request.node_namespace == '/ns_1'
    assert len(request.parameters) == 2
    assert request.parameters[0].name == 'param_1'
    assert request.parameters[0].value.integer_value == 11
    assert request.parameters[1].name == 'param_2'
    assert request.parameters[1].value.integer_value == 22

    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/ns_1/node_2') == 1
    assert request.node_name == 'node_2'
    assert request.node_namespace == '/ns_1'
    assert len(request.parameters) == 2
    assert request.parameters[0].name == 'param_3'
    assert request.parameters[0].value.integer_value == 33
    assert request.parameters[1].name == 'param_4'
    assert request.parameters[1].value.integer_value == 44

    # Case 8: node name not found
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='wrong_node_name',
            namespace='ns',
            parameters=[
                parameters_file_dir / 'example_parameters_no_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/ns/wrong_node_name') == 1
    assert request.node_name == 'wrong_node_name'
    assert request.node_namespace == '/ns'
    assert len(request.parameters) == 0

    # Case 9: wildcard mixed
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='my_node',
            namespace='/wildcard_ns/aa/extra1/extra2',
            parameters=[
                parameters_file_dir / 'example_parameters_wildcard_mixed.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/wildcard_ns/aa/extra1/extra2/my_node') == 1
    assert request.node_name == 'my_node'
    assert request.node_namespace == '/wildcard_ns/aa/extra1/extra2'
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'param'
    assert request.parameters[0].value.string_value == 'wildcard'

    # Namespace not found
    context = _assert_launch_no_errors([
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            namespace='foo',
            parameters=[
                parameters_file_dir / 'example_parameters_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/foo/test_node_name') == 1
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/foo'
    assert len(request.parameters) == 0

    # Node name with namespace from launch
    # Params file has no namespace
    context = _assert_launch_no_errors([
        PushROSNamespace('ns'),
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            parameters=[
                parameters_file_dir / 'example_parameters_no_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/ns/test_node_name') == 1
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/ns'
    assert len(request.parameters) == 0

    # Node name with namespace from launch
    # Params file has expected namespace
    context = _assert_launch_no_errors([
        PushROSNamespace('ns'),
        _load_composable_node(
            package='foo_package',
            plugin='bar_plugin',
            name='test_node_name',
            parameters=[
                parameters_file_dir / 'example_parameters_namespace.yaml'
            ],
        )
    ])
    request = mock_component_container.requests[-1]
    assert get_node_name_count(context, '/ns/test_node_name') == 1
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/ns'
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'param'
    assert request.parameters[0].value.integer_value == 1


def test_load_node_with_global_remaps_in_group(mock_component_container):
    """Test loading a node with global remaps scoped to a group."""
    context = _assert_launch_no_errors([
        GroupAction(
            [
                SetRemap('chatter', 'new_topic_name'),
                _load_composable_node(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='test_node_name',
                    namespace='test_node_namespace'
                ),
            ],
            scoped=True,
        ),
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/test_node_namespace/test_node_name') == 1

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/test_node_namespace'
    assert len(request.remap_rules) == 1
    assert request.remap_rules[0] == 'chatter:=new_topic_name'
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_namespace_in_group(mock_component_container):
    """Test loading a node with namespace scoped to a group."""
    context = _assert_launch_no_errors([
        GroupAction(
            [
                PushROSNamespace('foo'),
                _load_composable_node(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='test_node_name',
                    namespace='test_node_namespace'
                ),
            ],
            scoped=True,
        ),
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/foo/test_node_namespace/test_node_name') == 1

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name'
    assert request.node_namespace == '/foo/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0


def test_load_node_with_condition_in_group(mock_component_container):
    """Test loading nodes with conditions scoped to a group."""
    context = _assert_launch_no_errors([
        GroupAction(
            [
                PushROSNamespace('foo'),
                _load_composable_node(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='test_node_name_true',
                    namespace='test_node_namespace',
                    condition=IfCondition('True')
                ),
                _load_composable_node(
                    package='foo_package',
                    plugin='bar_plugin',
                    name='test_node_name_false',
                    namespace='test_node_namespace',
                    condition=IfCondition('False')
                ),
            ],
            scoped=True,
        ),
    ])

    # Check that launch is aware of loaded component
    assert get_node_name_count(context, '/foo/test_node_namespace/test_node_name_true') == 1
    assert get_node_name_count(context, '/foo/test_node_namespace/test_node_name_false') == 0

    # Check that container received correct request
    assert len(mock_component_container.requests) == 1
    request = mock_component_container.requests[0]
    assert request.package_name == 'foo_package'
    assert request.plugin_name == 'bar_plugin'
    assert request.node_name == 'test_node_name_true'
    assert request.node_namespace == '/foo/test_node_namespace'
    assert len(request.remap_rules) == 0
    assert len(request.parameters) == 0
    assert len(request.extra_arguments) == 0

def test_load_node_timeout_max_retries_error_message():
    """Test that RuntimeError on max retries includes node identity for debugging."""
    action = LoadComposableNodes(
        target_container=f'/{TEST_CONTAINER_NAME}',
        composable_node_descriptions=[
            ComposableNode(
                package='pointcloud_preprocessor',
                plugin='pointcloud_preprocessor::RingOutlierFilterComponent',
                name='ring_outlier_filter',
                namespace='sensing',
            )
        ]
    )

    # Set private attributes required by _resend_service_call_if_timeout
    mock_client = MagicMock()
    mock_client.srv_name = 'pointcloud_container/_container/load_node'
    mock_client.service_is_ready.return_value = True
    action._LoadComposableNodes__rclpy_load_node_client = mock_client
    action._LoadComposableNodes__final_target_container_name = 'pointcloud_container'

    request = LoadNode.Request()
    request.package_name = 'pointcloud_preprocessor'
    request.plugin_name = 'pointcloud_preprocessor::RingOutlierFilterComponent'
    request.node_name = 'ring_outlier_filter'
    request.node_namespace = '/sensing'

    mock_future = MagicMock()
    mock_event = threading.Event()
    context = LaunchContext()

    with patch('time.monotonic', return_value=100.0):
        with pytest.raises(RuntimeError) as exc_info:
            action._resend_service_call_if_timeout(
                response_future=mock_future,
                event=mock_event,
                attempt_started=0.0,
                timeout_sec=30.0,
                request=request,
                context=context,
                retry_count=10,
                max_retries=10,
            )

    err_msg = str(exc_info.value)
    assert 'ring_outlier_filter' in err_msg
    assert 'pointcloud_preprocessor' in err_msg
    assert 'RingOutlierFilterComponent' in err_msg
    assert 'pointcloud_container' in err_msg
    assert '10 retry attempts' in err_msg


def test_load_node_timeout_max_retries_error_message_unnamed_node():
    """Test RuntimeError message when node_name is empty (unnamed node)."""
    action = LoadComposableNodes(
        target_container=f'/{TEST_CONTAINER_NAME}',
        composable_node_descriptions=[
            ComposableNode(
                package='my_pkg',
                plugin='my_plugin::MyComponent',
            )
        ]
    )

    mock_client = MagicMock()
    mock_client.srv_name = 'container/_container/load_node'
    mock_client.service_is_ready.return_value = False
    action._LoadComposableNodes__rclpy_load_node_client = mock_client
    action._LoadComposableNodes__final_target_container_name = 'container'

    request = LoadNode.Request()
    request.package_name = 'my_pkg'
    request.plugin_name = 'my_plugin::MyComponent'
    request.node_name = ''
    request.node_namespace = '/'

    with patch('time.monotonic', return_value=100.0):
        with pytest.raises(RuntimeError) as exc_info:
            action._resend_service_call_if_timeout(
                response_future=MagicMock(),
                event=threading.Event(),
                attempt_started=0.0,
                timeout_sec=30.0,
                request=request,
                context=LaunchContext(),
                retry_count=10,
                max_retries=10,
            )

    err_msg = str(exc_info.value)
    assert '<unnamed>' in err_msg
    assert 'my_pkg' in err_msg
    assert 'MyComponent' in err_msg
    assert 'container' in err_msg


def test_load_node_timeout_retry_warning_message():
    """Test that retry path calls service_is_ready and resends the request."""
    action = LoadComposableNodes(
        target_container=f'/{TEST_CONTAINER_NAME}',
        composable_node_descriptions=[
            ComposableNode(
                package='foo_package',
                plugin='bar_plugin',
                name='test_node_name',
            )
        ]
    )

    mock_client = MagicMock()
    mock_client.srv_name = 'mock_component_container/_container/load_node'
    mock_client.service_is_ready.return_value = False
    mock_future = MagicMock()
    mock_client.call_async.return_value = mock_future
    action._LoadComposableNodes__rclpy_load_node_client = mock_client
    action._LoadComposableNodes__final_target_container_name = 'mock_component_container'

    request = LoadNode.Request()
    request.package_name = 'foo_package'
    request.plugin_name = 'bar_plugin'
    request.node_name = 'test_node_name'
    request.node_namespace = '/'

    mock_response_future = MagicMock()

    with patch('time.monotonic', return_value=100.0):
        result = action._resend_service_call_if_timeout(
            response_future=mock_response_future,
            event=threading.Event(),
            attempt_started=0.0,
            timeout_sec=30.0,
            request=request,
            context=LaunchContext(),
            retry_count=0,
            max_retries=10,
        )

    # Should have retried (returned new future, event, etc.)
    assert result[3] == 1  # new retry_count

    # Verify service_is_ready was called for diagnostic logging
    mock_client.service_is_ready.assert_called_once()

    # Verify call_async was invoked to resend the request
    mock_client.call_async.assert_called_once_with(request)

    # Verify original future was cancelled
    mock_response_future.cancel.assert_called_once()
