#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
import unique_id

import warehouse_ros as wr

from nav_msgs.msg import *
from nav_msgs.srv import *

from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *
from world_canvas_utils.serialization import *

from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid


def get_map(tmp):
	map_id = map_manager.list_maps().map_list[0].map_id
	resp = GetMapResponse()
	resp.map.info = map_manager.publish_map(map_id).info
	return resp


class MapManager:
	def __init__(self):
		self.map_collection = wr.MessageCollection('world_canvas', 'maps', OccupancyGrid)
		self.map_collection.ensure_index('uuid', unique=True)
		self.list_maps_srv = rospy.Service('list_maps', ListMaps, self.list_maps)
		self.publish_map_srv = rospy.Service('publish_map', PublishMap, self.publish_map)

	def list_maps(self):
		rospy.logdebug("Service call : list_maps")

		response = ListMapsResponse()

		all_maps = self.map_collection.query({}, metadata_only=True, sort_by='creation_time', ascending=False)

		# Loop over all maps metadata to get the first of each session.
		while True:
			try:
				map_md = all_maps.next()
				rospy.logdebug("Add map to result list: %s" % map_md)

				# Add the map info to our result list.
				new_entry = MapListEntry()
				new_entry.name = map_md.get('name', '')  # name is missing when auto-saving under-construction maps
				new_entry.date = map_md['creation_time']
				new_entry.session_id = map_md['session_id']
				new_entry.map_id = map_md['uuid']

				response.map_list.append(new_entry)
			except StopIteration:
				break

		return response

	def lookup_map(self, uuid):
		rospy.logdebug("Load map %s" % uuid)
		matching_maps = self.map_collection.query({'uuid': {'$in': [uuid]}})
		try:
			return matching_maps.next()[0]
		except StopIteration:
			rospy.logerr("No map found for uuid %s" % uuid)
			return None

	def publish_map(self, map_id):
		rospy.logdebug("Service call : publish_map %s" % map_id)

		map = self.lookup_map(map_id)
		return map


if __name__ == '__main__':
	rospy.init_node('static_map_test')

	static_map = OccupancyGrid()
	static_map.header.stamp = rospy.Time.now()
	static_map.header.frame_id = 'map'
	static_map.info.resolution = 0.1

	map_manager = MapManager()

	s = rospy.Service('/static_map', GetMap, get_map)

	rospy.spin()
