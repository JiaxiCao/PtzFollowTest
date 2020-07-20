#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PtzFollowTest.h"
#include <QtWidgets/QMainWindow>
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>
#include <QString>
#include <QTimer>
#include <QtWidgets>

#include <iostream>
#include <thread>
#include <mutex>
#include <fstream>

using namespace std;

class PtzFollowTest : public QMainWindow
{
    Q_OBJECT

public:
    PtzFollowTest(QWidget *parent = Q_NULLPTR);

private:
    Ui::PtzFollowTestClass ui;

	/**
	* @brief Serial port variables
	*/
	QSerialPort* serialport;
	QTimer* serial_timer;

	void CreateWindows();

	/**
	* @brief Serial port's dock window
	*/
	QDockWidget* dock_serial_port;
	void Init_serialport_dock();

	/**
	* @brief Serial port's buttons and combobox
	*/
	QLabel* label;
	QPushButton* scan_port, * close_port, * read_port;
	QComboBox* comport;

	/**
	@brief temp sapce and lock for imu data
	*/
	double imu_tmp_data[3];
	double imu_euler_data[3];
	std::mutex imu_read_mutex;
	std::condition_variable imu_read_condition;
	volatile bool imu_ready;

	double imu_euler_ref[3];

	volatile bool close_all;

private slots:
	void combox_activated();
	void read_serialport();
	void close_serialport();
	
private:
	void open_serialport();
	void read_IMU();
	void get_IMU_data();

	void open_PTZport();
	void Absolute_Rotation(double horizon, double vertical);
	void Relative_Rotation(double horizon, double vertical);
	void Reset();

	vector<double> pitch, roll, yaw;
	double pitch_mean, roll_mean, yaw_mean;
	QSerialPort* PTZport;
	double horizonal_PTZ_angle,vertical_PTZ_angle;
};
