#include "PtzFollowTest.h"

PtzFollowTest::PtzFollowTest(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	//delete main window
	QWidget* p = takeCentralWidget();
	if (p)
		delete p;
	setDockNestingEnabled(true);
	setWindowFlags(windowFlags() & ~Qt::WindowMaximizeButtonHint);

	CreateWindows();
}
void PtzFollowTest::CreateWindows()
{
	Init_serialport_dock();
	addDockWidget(Qt::LeftDockWidgetArea, dock_serial_port);
}

/*imu functions*/
void PtzFollowTest::Init_serialport_dock()
{
	imu_ready = false;
	close_all = false;
	horizonal_PTZ_angle = 0;
	vertical_PTZ_angle = 0;

	dock_serial_port = new QDockWidget(tr("串口"), this);
	dock_serial_port->setFeatures(QDockWidget::NoDockWidgetFeatures);
	dock_serial_port->setStyleSheet("QDockWidget:title{font-family:Microsoft YaHei;text-align: center}");
	dock_serial_port->setFixedWidth(440);
	dock_serial_port->setFixedHeight(80);

	serialport = new QSerialPort();
	label = new QLabel(tr("端口"), dock_serial_port);
	scan_port = new QPushButton(tr("扫描"), dock_serial_port);
	read_port = new QPushButton(tr("读取串口"), dock_serial_port);
	close_port = new QPushButton(tr("关闭"), dock_serial_port);
	comport = new QComboBox();

	label->setAlignment(Qt::AlignCenter);
	label->adjustSize();
	comport->adjustSize();
	scan_port->adjustSize();
	read_port->adjustSize();
	close_port->adjustSize();

	QHBoxLayout* hlayout1 = new QHBoxLayout();
	hlayout1->addWidget(label);
	hlayout1->addStretch();
	hlayout1->addWidget(comport);

	QHBoxLayout* hlayout2 = new QHBoxLayout();
	hlayout2->addLayout(hlayout1);
	hlayout2->addWidget(scan_port);
	hlayout2->addWidget(read_port);
	hlayout2->addWidget(close_port);

	QWidget* docklayout = new QWidget(dock_serial_port);
	docklayout->setLayout(hlayout2);
	dock_serial_port->setWidget(docklayout);

	//scanner serialport button
	connect(scan_port, SIGNAL(clicked()), this, SLOT(combox_activated()));
	//read serial port
	connect(read_port, SIGNAL(clicked()), this, SLOT(read_serialport()));
	//close serialport button
	connect(close_port, SIGNAL(clicked()), this, SLOT(close_serialport()));
}
void PtzFollowTest::open_serialport()
{
	//serialport->setPortName(comport->currentText());
	serialport->setPortName("COM4");
	QString data = comport->currentText();
	//qDebug() << data;
	serialport->setBaudRate(460800);
	serialport->setDataBits(QSerialPort::Data8);
	serialport->setParity(QSerialPort::NoParity);
	serialport->setStopBits(QSerialPort::OneStop);

	serialport->setFlowControl(QSerialPort::NoFlowControl);

	if (!serialport->open(QIODevice::ReadWrite))
	{
		QMessageBox::information(this, tr("提示"), tr("打开串口失败"));
	}
	else
	{
		QMessageBox::information(this, tr("提示"), tr("打开串口成功"));
	}
}
void PtzFollowTest::read_IMU()
{
	//open_serialport();
	std::ofstream fserial("IMUdata.csv");
	serialport->waitForReadyRead(10);
	serialport->readAll();

	/*find the first frame header in buffer*/
	int start = 0;
	while (start < 3)
	{
		serialport->waitForReadyRead(10);
		QByteArray buff_byte = serialport->read(1).toHex();
		int imu_byte = buff_byte.toInt(NULL, 16);

		if (start == 0)
		{
			if (imu_byte == 0x59) { start++; continue; }
		}
		if (start == 1)
		{
			if (imu_byte == 0x49) { start++; continue; }
			else { start = 0; continue; }
		}
		if (start == 2)
		{
			if (imu_byte == 0x53) { start++; continue; }
			else { start = 0; continue; }
		}
	}
	/*read the rest 124 bytes as imu reference pose*/
	serialport->waitForReadyRead(10);
	unsigned int buff_ref[124];
	for (int i = 0; i < 124; i++)
	{
		QByteArray btar;
		btar = serialport->read(1).toHex();
		while (btar.length() < 1)
		{
			serialport->waitForReadyRead(1);
			btar = serialport->read(1).toHex();
		}
		buff_ref[i] = btar.toInt(NULL, 16);
	}
	/*get reference euler angle*/
	for (int i = 0; i < 3; i++)
	{
		unsigned int euler = 0;
		for (int j = 0; j < 4; j++)
		{
			unsigned int buff = (unsigned int)buff_ref[75 + i * 4 + j];
			buff = buff << (j * 8);
			euler = buff | euler;
		}
		imu_euler_ref[i] = static_cast<int>(euler) * (1e-6);
		std::cout << imu_euler_ref[i]<<' ';
	}	

	int time_counter = 0;

	clock_t tim1 = clock();
	while (!close_all)
	{
		/*read a frame*/
		int imu_data[127];
		for (int i = 0; i < 127; i++)
		{
			QByteArray btar;
			btar = serialport->read(1).toHex();
			while (btar.length() < 1)
			{
				serialport->waitForReadyRead(1);
				btar = serialport->read(1).toHex();
			}
			imu_data[i] = btar.toInt(NULL, 16);
		}

		/*acceleration*/
		for (int i = 0; i < 3; i++)
		{
			unsigned int acce = 0;
			for (int j = 0; j < 4; j++)
			{
				unsigned int buff = (unsigned int)imu_data[8 + i * 4 + j];
				buff = buff << (j * 8);
				acce = buff | acce;
			}
			fserial << static_cast<int>(acce) * (1e-6) << ",";
		}
		/*free acceleration*/
		for (int i = 0; i < 3; i++)
		{
			unsigned int f_acce = 0;
			for (int j = 0; j < 4; j++)
			{
				unsigned int buff = (unsigned int)imu_data[22 + i * 4 + j];
				buff = buff << (j * 8);
				f_acce = buff | f_acce;
			}
			fserial << static_cast<int>(f_acce) * (1e-6) << ",";
		}
		/*delta velocity*/
		for (int i = 0; i < 3; i++)
		{
			unsigned int delta_v = 0;
			for (int j = 0; j < 4; j++)
			{
				unsigned int buff = (unsigned int)imu_data[36 + i * 4 + j];
				buff = buff << (j * 8);
				delta_v = buff | delta_v;
			}
			fserial << static_cast<int>(delta_v) * (1e-6) << ",";
		}
		/*angular velocity*/
		for (int i = 0; i < 3; i++)
		{
			unsigned int angle_v = 0;
			for (int j = 0; j < 4; j++)
			{
				unsigned int buff = (unsigned int)imu_data[50 + i * 4 + j];
				buff = buff << (j * 8);
				angle_v = buff | angle_v;
			}
			fserial << static_cast<int>(angle_v) * (1e-6) << ",";
		}
		/*magnetic*/
		for (int i = 0; i < 3; i++)
		{
			unsigned int mag = 0;
			for (int j = 0; j < 4; j++)
			{
				unsigned int buff = (unsigned int)imu_data[64 + i * 4 + j];
				buff = buff << (j * 8);
				mag = buff | mag;
			}
			fserial << static_cast<int>(mag) * (1e-6) << ",";
		}
		/*euler*/
		for (int i = 0; i < 3; i++)
		{
			unsigned int euler = 0;
			for (int j = 0; j < 4; j++)
			{
				unsigned int buff = (unsigned int)imu_data[78 + i * 4 + j];
				buff = buff << (j * 8);
				euler = buff | euler;
			}
			imu_tmp_data[i] = static_cast<int>(euler) * (1e-6);
			//fserial << static_cast<int>(euler) * (1e-6)<< ",";
			fserial << static_cast<int>(euler) * (1e-6) - imu_euler_ref[i] << ",";
		}
		fserial << std::endl;

		pitch.push_back(imu_tmp_data[0] - imu_euler_ref[0]);
		roll.push_back(imu_tmp_data[1] - imu_euler_ref[1]);
		yaw.push_back(imu_tmp_data[2] - imu_euler_ref[2]);
		if (pitch.size() > 10) pitch.erase(pitch.begin());
		if (roll.size() > 10) roll.erase(roll.begin());
		if (yaw.size() > 10) yaw.erase(yaw.begin());

		double pitch_tmp = 0.0, roll_tmp = 0.0, yaw_tmp = 0.0;
		for (int i = 0; i < pitch.size(); i++)
			pitch_tmp += pitch[i];
		if (pitch.size() > 0) pitch_tmp /= pitch.size();
		for (int i = 0; i < roll.size(); i++)
			roll_tmp += roll[i];
		if (roll.size() > 0) roll_tmp /= roll.size();
		for (int i = 0; i < yaw.size(); i++)
			yaw_tmp += yaw[i];
		if (yaw.size() > 0) yaw_tmp /= yaw.size();
		

		if (time_counter % 5 == 0)
		{
			std::unique_lock<std::mutex> putdata_lock(imu_read_mutex);

			//for (int i = 0; i < 3; i++)
			//	imu_euler_data[i] = imu_tmp_data[i];
			pitch_mean = pitch_tmp;
			roll_mean = roll_tmp;
			yaw_mean = yaw_tmp;
			imu_ready = true;

			putdata_lock.unlock();
			imu_read_condition.notify_all();
		}
		std::cout << time_counter++ << std::endl;

	}

	std::cout << double(clock() - tim1) / CLOCKS_PER_SEC << std::endl;

	fserial.close();
	return;

}
void PtzFollowTest::get_IMU_data()
{
	//open_PTZport();
	std::ofstream fsample("IMUdata_sample.csv");
	while (!close_all)
	{
		std::unique_lock<std::mutex> getdata_lock(imu_read_mutex);
		imu_read_condition.wait(getdata_lock, [=]() {return imu_ready; });
		imu_ready = false;
		/*fsample << imu_euler_data[0] - imu_euler_ref[0] << "," << imu_euler_data[1] - imu_euler_ref[1] << "," << imu_euler_data[2] - imu_euler_ref[2] << ",";
		pitch.push_back(imu_euler_data[0] - imu_euler_ref[0]);
		roll.push_back(imu_euler_data[1] - imu_euler_ref[1]);
		yaw.push_back(imu_euler_data[2] - imu_euler_ref[2]);
		lock_.unlock();

		if (pitch.size() > 10) pitch.erase(pitch.begin());
		if (roll.size() > 10) roll.erase(roll.begin());
		if (yaw.size() > 10) yaw.erase(yaw.begin());

		double pitch_mean = 0.0, roll_mean = 0.0, yaw_mean = 0.0;
		for (int i = 0; i < pitch.size(); i++)
			pitch_mean += pitch[i];
		if (pitch.size() > 0) pitch_mean /= pitch.size();
		for (int i = 0; i < roll.size(); i++)
			roll_mean += roll[i];
		if (roll.size() > 0) roll_mean /= roll.size();
		for (int i = 0; i < yaw.size(); i++)
			yaw_mean += yaw[i];
		if (yaw.size() > 0) yaw_mean /= yaw.size();*/
		fsample << pitch_mean << "," << roll_mean << "," << yaw_mean << "," << std::endl;
		if (abs(pitch_mean - vertical_PTZ_angle) > 0.5 || abs(yaw_mean - horizonal_PTZ_angle) > 0.5)
		{
			vertical_PTZ_angle = pitch_mean;
			horizonal_PTZ_angle = yaw_mean;
			Absolute_Rotation(horizonal_PTZ_angle, vertical_PTZ_angle);
		}
		getdata_lock.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	fsample.close();
}

/*slots*/
void PtzFollowTest::combox_activated()
{
	comport->clear();
	foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts())
	{
		qDebug() << info.portName() << endl;
		comport->addItem(info.portName());
	}
	QMessageBox::information(this, tr("串口信息"), tr("扫描串口"));
}
void PtzFollowTest::read_serialport()
{
	open_serialport();
	open_PTZport();
	/*read data from buffer*/
	std::thread ca(std::bind(&PtzFollowTest::read_IMU, this));
	ca.detach();
	/*sample and filt*/
	std::thread gd(std::bind(&PtzFollowTest::get_IMU_data, this));
	gd.detach();
}
void PtzFollowTest::close_serialport()
{
	Reset();
	close_all = true;
	QMessageBox::information(this, tr("提示"), tr("关闭串口"));
	serialport->close();
}

/*ptz functions*/
void PtzFollowTest::open_PTZport()
{
	PTZport = new QSerialPort();
	PTZport->setPortName("COM6");
	QString data = comport->currentText();
	PTZport->setBaudRate(115200);
	PTZport->setDataBits(QSerialPort::Data8);
	PTZport->setParity(QSerialPort::NoParity);
	PTZport->setStopBits(QSerialPort::OneStop);

	PTZport->setFlowControl(QSerialPort::NoFlowControl);

	if (!PTZport->open(QIODevice::ReadWrite))
	{
		QMessageBox::information(this, tr("提示"), tr("打开云台失败"));
	}
	else
	{
		QMessageBox::information(this, tr("提示"), tr("打开云台成功"));
	}
}
void PtzFollowTest::Reset()
{
	QString str;
	str.append("set1:0.0,0.0,0.0,0.0\r\n");
	PTZport->write(str.toLatin1());
	PTZport->waitForBytesWritten(-1);
}
void PtzFollowTest::Absolute_Rotation(double horizon, double vertical)
{
	QString str;
	str.append("set1:");
	str.append(QString::number(int(horizon / 0.00625) * 0.00625));
	str.append(",");
	str.append(QString::number(int(vertical / 0.00625) * 0.00625));
	str.append(",0.0,0.0\r\n");

	PTZport->write(str.toLatin1());
	PTZport->waitForBytesWritten(-1);
}
void PtzFollowTest::Relative_Rotation(double horizon, double vertical)
{

	QString str;
	str.append("set1:");
	str.append(QString::number(int((horizonal_PTZ_angle + horizon) / 0.00625) * 0.00625));
	str.append(",");
	str.append(QString::number(int((vertical_PTZ_angle + vertical) / 0.00625) * 0.00625));
	str.append(",0.0,0.0\r\n");

	PTZport->write(str.toLatin1());
	PTZport->waitForBytesWritten(-1);
}


