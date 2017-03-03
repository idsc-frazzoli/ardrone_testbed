#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import sys
import unittest

import uuid                     # standard Python module

from uuid_msgs.msg import UniqueID
from unique_id import *

class TestPythonUUID(unittest.TestCase):
    """Unit tests for Python UUID generation.
    """

    # random UUID generation tests
    def test_random_uuids(self):
        N = 1000
        uu = []
        for i in xrange(N):
            uu.append(fromRandom())
            self.assertEqual(type(uu[i]), uuid.UUID)
            for j in xrange(i-1, -1, -1):
                self.assertNotEqual(uu[i], uu[j])

    # UUID generation from URL tests
    def test_empty_url(self):
        x = fromURL('')
        self.assertEqual(type(x), uuid.UUID)
        y = fromURL('')
        self.assertEqual(type(y), uuid.UUID)
        self.assertEqual(x, y)
        # MUST yield same result as C++ fromURL() function:
        self.assertEqual(str(x), "1b4db7eb-4057-5ddf-91e0-36dec72071f5")

    def test_same_url(self):
        x = fromURL('http://openstreetmap.org/node/1')
        self.assertEqual(type(x), uuid.UUID)
        y = fromURL('http://openstreetmap.org/node/1')
        self.assertEqual(type(y), uuid.UUID)
        self.assertEqual(x, y)
        # MUST yield same result as C++ fromURL() function:
        self.assertEqual(str(x), 'ef362ac8-9659-5481-b954-88e9b741c8f9')

    def test_same_id_different_osm_namespace(self):
        x = fromURL('http://openstreetmap.org/node/1')
        y = fromURL('http://openstreetmap.org/way/1')
        self.assertNotEqual(x, y)
        # MUST yield same result as C++ fromURL() function:
        self.assertEqual(str(y), 'b3180681-b125-5e41-bd04-3c8b046175b4')

    def test_actual_osm_node_id(self):
        x = fromURL('http://openstreetmap.org/node/1')
        y = fromURL('http://openstreetmap.org/node/152370223')
        self.assertNotEqual(x, y)
        # MUST yield same result as C++ fromURL() function:
        self.assertEqual(str(y), '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac')

    def test_route_segment(self):
        start = 'da7c242f-2efe-5175-9961-49cc621b80b9'
        end = '812f1c08-a34b-5a21-92b9-18b2b0cf4950'
        x = fromURL('http://ros.org/wiki/road_network/' + start + '/' + end)
        y = fromURL('http://ros.org/wiki/road_network/' + end + '/' + start)
        self.assertNotEqual(x, y)
        # MUST yield same result as C++ fromURL() function:
        self.assertEqual(str(x), 'acaa906e-8411-5b45-a446-ccdc2fc39f29')

    # UniqueID message generation tests
    def test_msg_creation(self):
        msg = toMsg(fromURL('http://openstreetmap.org/node/152370223'))
        self.assertEqual(toHexString(msg),
                         '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac')
        
    def test_msg_same_id_different_namespace(self):
        x = toMsg(fromURL('http://openstreetmap.org/node/1'))
        y = toMsg(fromURL('http://openstreetmap.org/way/1'))
        self.assertNotEqual(x, y)
        self.assertEqual(toHexString(y),
                         'b3180681-b125-5e41-bd04-3c8b046175b4')

    def test_msg_route_segment(self):
        start = 'da7c242f-2efe-5175-9961-49cc621b80b9'
        end = '812f1c08-a34b-5a21-92b9-18b2b0cf4950'
        x = toMsg(fromURL('http://ros.org/wiki/road_network/'
                          + start + '/' + end))
        y = toMsg(fromURL('http://ros.org/wiki/road_network/'
                          + end + '/' + start))
        self.assertNotEqual(x, y)
        self.assertEqual(toHexString(x),
                         'acaa906e-8411-5b45-a446-ccdc2fc39f29')

    def test_nil_msg(self):
        x = UniqueID()
        y = toMsg(uuid.UUID(hex='00000000-0000-0000-0000-000000000000'))
        self.assertEqual(x, y)

    def test_random_msg(self):
        x = UniqueID()
        y = toMsg(fromRandom())
        self.assertNotEqual(x, y)

    def test_equivalent_msgs(self):
        s = 'da7c242f-2efe-5175-9961-49cc621b80b9'
        x = toMsg(uuid.UUID(s))
        y = toMsg(uuid.UUID(s))
        self.assertEqual(x, y)
        self.assertEqual(s, toHexString(y))

    def test_to_and_from_msg(self):
        x = uuid.UUID('da7c242f-2efe-5175-9961-49cc621b80b9')
        y = (fromMsg(toMsg(x)))
        self.assertEqual(x, y)
        self.assertEqual(type(y), uuid.UUID)

    def test_msg_to_string(self):
        s = 'da7c242f-2efe-5175-9961-49cc621b80b9'
        x = toMsg(uuid.UUID(s))
        y = toHexString(x)
        self.assertEqual(s, y)
        self.assertEqual(type(y), str)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('unique_id', 'test_uuid_py', TestPythonUUID) 
