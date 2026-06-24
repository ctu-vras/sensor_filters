.. SPDX-License-Identifier: BSD-3-Clause
.. SPDX-FileCopyrightText: Czech Technical University in Prague

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sensor_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added integration tests.
* Fixed license issues, made the repo REUSE-compliant.
* Added support for lazy input topics.
* Added support for content filters on subscriber.
* Allow overriding QoS settings.
* Add new message types for joy feedback array, fluid pressure, illuminance and other
* Use node interfaces in transport-based chains if available.
* Add image and pointcloud2 filter chains
* Add lifecycle nodes
* Port to ROS 2
* Contributors: Martin Pecka, solonovamax

1.1.1 (2023-06-07)
------------------
* Reformatted, added catkin lint.
* Simplified the license statements.
* Contributors: Martin Pecka

1.1.0 (2023-05-29)
------------------
* Using image_transport and point_cloud_transport where applicable.
* Noetic compatibility.
* Improved README and added example files.
  Closes `#1 <https://github.com/ctu-vras/sensor_filters/issues/1>`_.
* Contributors: Martin Pecka

1.0.5 (2021-07-30)
------------------
* Fixed typo leading to segfaults when using ChangeHeader filter.
* Contributors: Martin Pecka

1.0.4 (2021-06-24)
------------------
* Added ChangeHeader filter
* Contributors: Martin Pecka

1.0.3 (2021-05-20)
------------------
* Fixed data types of nodes.
* Contributors: Martin Pecka

1.0.2 (2021-05-20)
------------------
* Fix for Noetic.
* Contributors: Martin Pecka

1.0.1 (2021-05-19)
------------------
* Initial version.
* Contributors: Martin Pecka
