0.3.1 (2024-06-07)
------------------

* Fix: parameter 'camera_wb_red' cannot be set
* Fixed limiting float parameters to avoid errors due to rounding
* Fix: don't treat unknown parameters as error in the parameter callback (e.g. for new enable_pub_plugins param injected by image_transport)
* Fix: Declaring parameters dynamic so that undeclaring works on cleanup

0.3.0 (2023-03-10)
------------------

* Replaced parameter camera_exp_auto by camera_exp_control and camera_exp_auto_mode for consistency
* Change parameters camera_exp_max and camera_exp_value so unit seconds for consistency
* Added Gamma parameter
* only publish high/low if the publisher exists
* fix if iocontrol is not available

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
