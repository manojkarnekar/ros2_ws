
INSERT INTO waypoint_types (Id, CreatedOn, Name) VALUES (0, '2020-01-24 18:16:29.300733', 'Default');
INSERT INTO waypoint_types (Id, CreatedOn, Name) VALUES (1, '2020-01-24 18:16:29.300747', 'Target');
INSERT INTO waypoint_types (Id, CreatedOn, Name) VALUES (2, '2020-01-24 18:16:29.300745', 'ElevatorArea');
INSERT INTO waypoint_types (Id, CreatedOn, Name) VALUES (3, '2020-01-24 18:16:29.300745', 'Elevator');
INSERT INTO waypoint_types (Id, CreatedOn, Name) VALUES (4, '2020-01-24 18:16:29.300745', 'ChargingArea');
INSERT INTO waypoint_types (Id, CreatedOn, Name) VALUES (5, '2020-01-24 18:16:29.300745', 'DeliveryTarget');

INSERT INTO device_stats (Id, CreatedOn, UpdatedOn, Name, Value) VALUES (0, '2020-01-24 18:16:29.302614', '2020-01-24 18:16:29.302624', 'battery', '0');
INSERT INTO device_stats (Id, CreatedOn, UpdatedOn, Name, Value) VALUES (1, '2020-01-24 18:16:29.302627', '2020-01-24 18:16:29.302628', 'lidar', '0');
INSERT INTO device_stats (Id, CreatedOn, UpdatedOn, Name, Value) VALUES (2, '2020-01-24 18:16:29.302630', '2020-01-24 18:16:29.302631', 'core', '0');
INSERT INTO device_stats (Id, CreatedOn, UpdatedOn, Name, Value) VALUES (3, '2020-01-24 18:16:29.302633', '2020-01-24 18:16:29.302635', 'arduino1', '0');
INSERT INTO device_stats (Id, CreatedOn, UpdatedOn, Name, Value) VALUES (4, '2020-01-24 18:16:29.302636', '2020-01-24 18:16:29.302638', 'manual_mode', '0');
INSERT INTO device_stats (Id, CreatedOn, UpdatedOn, Name, Value) VALUES (5, '2020-01-24 18:16:29.302639', '2020-01-24 18:16:29.302641', 'move_state', '0');

INSERT INTO status (Id, CreatedOn, Status) VALUES (0, '2020-01-24 18:16:29.297490', 'UNINITIALIZED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (10, '2020-01-24 18:16:29.297498', 'INITIALIZED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (20, '2020-01-24 18:16:29.297499', 'READY');
INSERT INTO status (Id, CreatedOn, Status) VALUES (30, '2020-01-24 18:16:29.297500', 'DISPATCH_AUTH');
INSERT INTO status (Id, CreatedOn, Status) VALUES (40, '2020-01-24 18:16:29.297502', 'INVENTORY_SCAN_IN');
INSERT INTO status (Id, CreatedOn, Status) VALUES (50, '2020-01-24 18:16:29.297503', 'DOOR_CLOSED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (60, '2020-01-24 18:16:29.297504', 'WAYPOINT_RECEIVED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (70, '2020-01-24 18:16:29.297504', 'MOVING');
INSERT INTO status (Id, CreatedOn, Status) VALUES (80, '2020-01-24 18:16:29.297505', 'GOAL_REACHED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (90, '2020-01-24 18:16:29.297507', 'DELIVERY_AUTH');
INSERT INTO status (Id, CreatedOn, Status) VALUES (100, '2020-01-24 18:16:29.297507', 'INVENTORY_SCAN_OUT');
INSERT INTO status (Id, CreatedOn, Status) VALUES (110, '2020-01-24 18:16:29.297508', 'DELIVERY_COMPLETE');
INSERT INTO status (Id, CreatedOn, Status) VALUES (120, '2020-01-24 18:16:29.297509', 'DELIVERY_CANCELLED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (130, '2020-01-24 18:16:29.297510', 'NAVIGATION_PAUSED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (140, '2020-01-24 18:16:29.297511', 'MOVE_ERROR');
INSERT INTO status (Id, CreatedOn, Status) VALUES (150, '2020-01-24 18:16:29.297512', 'HARDWARE_ERROR');
INSERT INTO status (Id, CreatedOn, Status) VALUES (160, '2020-01-24 18:16:29.297513', 'BATTERY_OUT');
INSERT INTO status (Id, CreatedOn, Status) VALUES (162, '2020-01-24 18:16:29.297514', 'BATTERY_OUT_MOVING');
INSERT INTO status (Id, CreatedOn, Status) VALUES (164, '2020-01-24 18:16:29.297517', 'BATTERY_OUT_POSITIONED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (170, '2020-01-24 18:16:29.297518', 'CHARGING');
INSERT INTO status (Id, CreatedOn, Status) VALUES (180, '2020-01-24 18:16:29.297519', 'ELEVATOR_CALL');
INSERT INTO status (Id, CreatedOn, Status) VALUES (190, '2020-01-24 18:16:29.297520', 'ELEVATOR_MOVING_IN');
INSERT INTO status (Id, CreatedOn, Status) VALUES (200, '2020-01-24 18:16:29.297521', 'ELEVATOR_IN');
INSERT INTO status (Id, CreatedOn, Status) VALUES (210, '2020-01-24 18:16:29.297522', 'ELEVATOR_MOVING');
INSERT INTO status (Id, CreatedOn, Status) VALUES (220, '2020-01-24 18:16:29.297523', 'ELEVATOR_REACHED');
INSERT INTO status (Id, CreatedOn, Status) VALUES (230, '2020-01-24 18:16:29.297524', 'ELEVATOR_MOVING_OUT');
INSERT INTO status (Id, CreatedOn, Status) VALUES (240, '2020-01-24 18:16:29.297525', 'ELEVATOR_OUT');




INSERT INTO amro_trips (Id, CreatedOn, PathId, InitiatedBy, RecievedBy, StatusId) VALUES (1, '2020-01-24 18:16:29.306842', '', '', '', 10);

INSERT INTO users (Id, CreatedOn, Name, Passcode) VALUES (1, '2020-01-24 18:16:29.334810', 'Ranjit', '123');
INSERT INTO users (Id, CreatedOn, Name, Passcode) VALUES (2, '2020-01-24 18:16:29.334825', 'Akhil', '124');
INSERT INTO users (Id, CreatedOn, Name, Passcode) VALUES (3, '2020-01-24 18:16:29.334828', 'Manish', '125');
 
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (1, '2020-01-24 18:16:29.310951', 'Celecoxib', '', 1, '9804966096', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (2, '2020-01-24 18:16:29.310971', 'Dolo 650', '', 3, '9176541848', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (3, '2020-01-24 18:16:29.310976', 'Hydroxyurea', '', 2, '9782701582', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (4, '2020-01-24 18:16:29.310980', 'Labetalol', '', 1, '7044389887', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (5, '2020-01-24 18:16:29.310984', 'Mitotane', '', 2, '9785645892', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (6, '2020-01-24 18:16:29.310987', 'Phenotoin', '', 1, '7564856529', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (7, '2020-01-24 18:16:29.310990', 'Rituximb', '', 2, '8515647562', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (8, '2020-01-24 18:16:29.310995', 'Sunitinib', '', 3, '8692365482', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (9, '2020-01-24 18:16:29.310999', 'Topotcan', '', 4, '8945751575', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (10, '2020-01-24 18:16:29.311003', 'Vorinostat', '', 1, '6965852574', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (11, '2020-01-24 18:16:29.311006', 'Warfarin', '', 1, '9632587412', '');
INSERT INTO stock (Id, CreatedOn, Name, Description, Quantity, Barcode, Image) VALUES (12, '2020-01-24 18:16:29.311009', 'Zidovudine', '', 1, '7315935745', '');
 