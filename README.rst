LCSR TF Tools
=============

This package contains a bunch of utilities that have been created in the aim to
fill some usability gaps in TF.

Tools
=====

Multi-Static Publisher
----------------------
The most widely-usable tool in this package is the
mutli_static_transform_publisher. This node listens on a topic for static
transforms to publish, and then re-publishes them as batch messages when it can
do so. It will publish at the highest common period of the requested frames.

The publisher itself has no arguments, but instead it is controlled by running
two other scripts:
- set_multi_static.py
- del_multi_static.py


Usage::
  set_multi_static.py [-h] [--node-name node_name] [-xyz x y z]
                             [-aa x y z t | -ypr yaw pitch roll | -quat qx qy qz qw]
                             frame_id child_frame_id period

  Set frames being broadcast by a multi_static_transform_publisher

  positional arguments:
    frame_id              The frame_id of the frame in which new new frame is
                          defined.
    child_frame_id        The frame_id of the new frame.
    period                Publish period in ms.

  optional arguments:
    -h, --help            show this help message and exit
    --node-name node_name
                          The name of the multi publisher that should publish
                          this transform. Default is 'multi_tf_pub'
    -xyz x y z            Position in x, y, z)
    -aa x y z t           Orientation in axis/angle x, y, z, theta
    -ypr yaw pitch roll   Orientation in yaw, pitch, roll
    -quat qx qy qz qw     Orientation in quaternion



Usage::
  del_multi_static.py [-h] [--node-name node_name]
                             frame_id [frame_id ...]

  Delete frames being broadcast by a multi_static_transform_publisher

  positional arguments:
    frame_id              The frame_ids of each frame to stop broadcasting.

  optional arguments:
    -h, --help            show this help message and exit
    --node-name node_name
                          The name of the multi publisher that should publish
                          this transform. Default is 'multi_tf_pub'

