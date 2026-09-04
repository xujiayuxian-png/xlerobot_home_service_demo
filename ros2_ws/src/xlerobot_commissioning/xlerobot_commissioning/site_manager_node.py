"""ROS service boundary for staging and activating site bundles."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from xlerobot_commissioning.site_assets import SiteDraftStore
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import ActivateArtifact, RecordSiteValidation
from xlerobot_interfaces.srv import RemoveNamedPlace, SaveSiteMap, SetNamedPlace


class SiteManagerNode(Node):
    """Keep mutable site drafts and immutable catalog publication out of HMI."""

    def __init__(self):
        super().__init__('site_manager')
        root = self.declare_parameter(
            'artifact_root', '~/.local/share/xlerobot'
        ).value
        revision = self.declare_parameter('software_revision', 'development').value
        operator = self.declare_parameter('operator', 'technician').value
        self.store = SiteDraftStore(
            Path(str(root)).expanduser(), software_revision=str(revision),
            operator=str(operator),
        )
        self.create_service(SaveSiteMap, '/site_manager/save_map', self.save_map)
        self.create_service(SetNamedPlace, '/site_manager/set_place', self.set_place)
        self.create_service(
            RemoveNamedPlace, '/site_manager/remove_place', self.remove_place
        )
        self.create_service(
            RecordSiteValidation, '/site_manager/record_validation',
            self.record_validation,
        )
        self.create_service(
            ActivateArtifact, '/site_manager/activate', self.activate
        )

    @staticmethod
    def _error(response, error: Exception):
        response.error.code = (
            CapabilityError.NOT_FOUND
            if isinstance(error, (FileNotFoundError, KeyError))
            else CapabilityError.INVALID_GOAL
        )
        response.error.message = str(error)
        return response

    def save_map(self, request, response):
        try:
            draft = self.store.save_map(
                request.site_id, request.map_name, request.source_map_yaml
            )
            response.error.code = CapabilityError.NONE
            response.error.message = 'map copied into site draft'
            response.artifact_uri = draft.as_uri()
            response.version = 'draft'
        except Exception as error:  # services return typed errors, never crash executor
            self._error(response, error)
        return response

    def set_place(self, request, response):
        try:
            quaternion = request.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
            )
            draft = self.store.set_place(
                request.site_id, request.place_id,
                frame_id=request.pose.header.frame_id or 'map',
                x=request.pose.pose.position.x, y=request.pose.pose.position.y,
                yaw=yaw, nav_offset_m=request.nav_offset_m, dock=request.dock,
            )
            response.error.code = CapabilityError.NONE
            response.error.message = 'named place saved in site draft'
            response.artifact_uri = draft.as_uri()
        except Exception as error:
            self._error(response, error)
        return response

    def record_validation(self, request, response):
        try:
            response.site_ready = self.store.record_validation(
                request.site_id, request.place_id,
                localization_passed=request.localization_passed,
                navigation_passed=request.navigation_passed,
                dock_passed=request.dock_passed,
                position_error_m=request.position_error_m,
                yaw_error_rad=request.yaw_error_rad,
                duration_s=request.duration_s, message=request.message,
            )
            response.error.code = CapabilityError.NONE
            response.error.message = 'site validation recorded'
            response.artifact_uri = self.store.draft(request.site_id).as_uri()
        except Exception as error:
            self._error(response, error)
        return response

    def remove_place(self, request, response):
        try:
            draft = self.store.remove_place(request.site_id, request.place_id)
            response.error.code = CapabilityError.NONE
            response.error.message = 'named place removed from site draft'
            response.artifact_uri = draft.as_uri()
        except Exception as error:
            self._error(response, error)
        return response

    def activate(self, request, response):
        try:
            if request.artifact_type != 'site':
                raise ValueError('site manager only activates artifact_type=site')
            if request.version:
                reference = self.store.catalog.activate(
                    'sites', request.artifact_id, request.version
                )
            else:
                reference = self.store.finalize_and_activate(request.artifact_id)
            response.error.code = CapabilityError.NONE
            response.error.message = 'site bundle activated'
            response.artifact_uri = reference.uri
        except Exception as error:
            self._error(response, error)
        return response


def main():
    rclpy.init()
    node = SiteManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
