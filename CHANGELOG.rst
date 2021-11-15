0.2.1 (2021-11-15)
------------------

* Add exposure_adapt_timeout parameter
* warn instead of info if parameter can't be set
* fix line_source and add more extra_data to CameraParam

0.2.0 (2021-08-02)
------------------

For supporting rc_sgm_producer:

* Extend color format to RGB8 for ros foxy
* Using parameter PtpEnable instead of deprecated parameter GevIEEE1588

0.1.4 (2021-02-11)
------------------

* Accept devices from vendor Roboception or if model is rc_visard or rc_cube

0.1.3 (2021-02-02)
------------------

* Use image_transport.hpp to get rid of a warning and hence drop eloquent support
* Use C-strings for RCLCPP macros to support rolling

0.1.2 (2021-01-29)
------------------

* fix linking problems

0.1.1 (2021-01-18)
------------------

* fix typo in package xml

0.1.0 (2021-01-18)
------------------

* initial release
