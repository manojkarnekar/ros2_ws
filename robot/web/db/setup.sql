--
-- File generated with SQLiteStudio v3.2.1 on Sat Jan 25 21:42:55 2020
--
-- Text encoding used: UTF-8
--
PRAGMA foreign_keys = off;
BEGIN TRANSACTION;

-- Table: amro_stock
DROP TABLE IF EXISTS amro_stock;
CREATE TABLE "amro_stock" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "StockId"    INTEGER NOT NULL,
            "TripId"    INTEGER NOT NULL,
            "Quantity"    INTEGER DEFAULT 1
        );

-- Table: amro_trips
DROP TABLE IF EXISTS amro_trips;
CREATE TABLE "amro_trips" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "PathId"    INTEGER,
            "InitiatedBy"    INTEGER NOT NULL,
            "RecievedBy"    INTEGER,
            "StatusId"    INTEGER
        );

-- Table: device_stats
DROP TABLE IF EXISTS device_stats;
CREATE TABLE "device_stats" (
            "Id"    INTEGER NOT NULL PRIMARY KEY,
            "CreatedOn"    TEXT NOT NULL,
            "UpdatedOn"    TEXT NOT NULL,
            "Name"    TEXT NOT NULL,
            "Value"    TEXT NOT NULL
        );

-- Table: floors
DROP TABLE IF EXISTS floors;
CREATE TABLE "floors" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "Name"    TEXT,
            "Description"    TEXT,
            "BuildingId"    INTEGER,
            "IsActive"    INTEGER
        );

-- Table: paths
DROP TABLE IF EXISTS paths;
CREATE TABLE "paths" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "WaypointIdA"    INTEGER NOT NULL,
            "WaypointIdB"    INTEGER NOT NULL,
            "PathString"    TEXT,
            "isActive"    INTEGER DEFAULT 0
        );

-- Table: status
DROP TABLE IF EXISTS status;
CREATE TABLE "status" (
            "Id"    INTEGER NOT NULL PRIMARY KEY,
            "CreatedOn"    TEXT NOT NULL,
            "Status"    TEXT NOT NULL
        );

-- Table: stock
DROP TABLE IF EXISTS stock;
CREATE TABLE "stock" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "Name"    TEXT NOT NULL,
            "Description"    TEXT,
            "Quantity"    INTEGER DEFAULT 1,
            "Barcode"    TEXT NOT NULL UNIQUE,
            "Image"    TEXT
        );

-- Table: users
DROP TABLE IF EXISTS users;
CREATE TABLE "users" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "Name"    TEXT NOT NULL,
            "Passcode"    TEXT NOT NULL
        );

-- Table: waypoint_types
DROP TABLE IF EXISTS waypoint_types;
CREATE TABLE "waypoint_types" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "Name"    TEXT NOT NULL
        );

-- Table: waypoints
DROP TABLE IF EXISTS waypoints;
CREATE TABLE "waypoints" (
            "Id"    INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
            "CreatedOn"    TEXT NOT NULL,
            "Type"    INTEGER NOT NULL DEFAULT 0,
            "Name"    TEXT NOT NULL,
            "FloorId"    INTEGER NOT NULL,
            "PosX"    REAL,
            "PosY"    REAL,
            "PosZ"    REAL,
            "PosW"    REAL,
            "isActive"    INTEGER DEFAULT 0
        );

COMMIT TRANSACTION;
PRAGMA foreign_keys = on;
