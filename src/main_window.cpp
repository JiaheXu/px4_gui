#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h> 
#include "../include/px4_gui/main_window.hpp"


#define max_height 4.0

namespace px4_gui {

using namespace Qt;
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindowDesign),
      qnode(argc,argv)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/AE.png"));
    ui->tab_manager->setCurrentIndex(0); 
    // update some GUI components for online status
    ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
    ui->label_statue_text->setStyleSheet("color:green;");
    ui->label_statue_text->setText("online");
    qnode.init();
    // get current topics
    initTopicList();

    start_from_the_beginning();
}
void MainWindow::start_from_the_beginning()
{
//decide whether to polt row 3 & 4 . 0 for no ,1 for yes
    plotting_row34 = int(ui->extra_plots_btn->isChecked()) ;
    disconnections();

// since xAxis is controlled by the coord we give to plot() function
// for simplicity we just record offset(last time we click start button)
// and make the timeline back to 0
    offset = lastPointKey1;

    initQcustomplot();
    setLegend();
    TickerTime();
    connections();
}
MainWindow::~MainWindow()
{
    stop_recording();
}
void MainWindow::ROSRUN()
{
    qnode.run();
}

void MainWindow::initQcustomplot()
{
qDebug() << "plots!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    ui->horizontalScrollBar_1->setPageStep( 30 );

//https://color.adobe.com/zh/create/color-wheel
// QColor(255, 80, 255) bright purple *
// QColor(255, 118, 56) orange *

//colors
//51 255 48 
//222 185 42
//245 93 59
//144 42 222
//0 188 255
//255 62 249

// wide lines(width 4) alpha is 200 
// narrow dotline (width 2) alpha is 255
QPen pen1,pen2,pen3,pen4,pen5;

pen1.setColor(QColor(51, 255, 48 , 255));
pen1.setStyle(Qt::DotLine);
pen1.setWidthF(2);

pen2.setColor(QColor(222, 185, 42, 255));
pen2.setStyle(Qt::DotLine);
pen2.setWidthF(2);

pen3.setColor(QColor(245, 93, 59, 255));
pen3.setStyle(Qt::DotLine);
pen3.setWidthF(2);

pen4.setColor(QColor(144, 42, 222, 255));
pen4.setStyle(Qt::DotLine);
pen4.setWidthF(2);

pen5.setColor(QColor(255, 0, 70, 255));
pen5.setStyle(Qt::DotLine);
pen5.setWidthF(2);


QPen linePen1;
linePen1.setColor(QColor(0, 188, 255, 150));
linePen1.setWidthF(4);

QPen linePen2;
linePen2.setColor(QColor(255, 118, 56, 150));
linePen2.setWidthF(4);

QPen redPen;
redPen.setColor(QColor(255, 0, 0, 255));
redPen.setStyle(Qt::DotLine);
redPen.setWidthF(2);

QPen greenPen;
greenPen.setColor(QColor(15, 219 , 21, 255));
//greenPen.setColor(QColor(0, 255 , 0, 255));
greenPen.setStyle(Qt::DotLine);
greenPen.setWidthF(2);

QPen bluePen;
bluePen.setColor(QColor(0 , 0 , 255 , 255));
bluePen.setStyle(Qt::DotLine);
bluePen.setWidthF(2);


//pos 1234
//goal 1234
//pose setpoint 1234
//vel_yaw 4


ui->plot_1_1->clearGraphs();
ui->plot_1_2->clearGraphs();
ui->plot_1_3->clearGraphs();
ui->plot_1_4->clearGraphs();
ui->plot_2_1->clearGraphs();
ui->plot_2_2->clearGraphs();
ui->plot_2_3->clearGraphs();
ui->plot_2_4->clearGraphs();
ui->plot_3_1->clearGraphs();
ui->plot_3_2->clearGraphs();
ui->plot_3_3->clearGraphs();
ui->plot_4_1->clearGraphs();
ui->plot_4_2->clearGraphs();
ui->plot_4_3->clearGraphs();

// row 1
    

    ui->plot_1_1->addGraph(); // red line
    ui->plot_1_1->graph(0)->setPen(redPen); //pose x
    //reference line
    ui->plot_1_1->addGraph(); // bright purple line
    ui->plot_1_1->graph(1)->setPen(linePen1); // goal pose x
    ui->plot_1_1->addGraph();
    ui->plot_1_1->graph(2)->setPen(linePen2); // pose setpoint x

    ui->plot_1_1->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_1_1->axisRect()->setupFullAxesBox(true);
    ui->plot_1_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_1_1->setRange(-100, 100);


    ui->plot_1_2->addGraph(); // green line
    ui->plot_1_2->graph(0)->setPen(greenPen); //pose y
    //reference line
    ui->plot_1_2->addGraph(); // bright purple line
    ui->plot_1_2->graph(1)->setPen(linePen1); // goal pose y
    ui->plot_1_2->addGraph();
    ui->plot_1_2->graph(2)->setPen(linePen2);  // pose setpoint y

    ui->plot_1_2->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_1_2->axisRect()->setupFullAxesBox(true);
    ui->plot_1_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_1_2->setRange(-100, 100);

    ui->plot_1_3->addGraph(); // blue line
    ui->plot_1_3->graph(0)->setPen(bluePen); //pose z
    //reference line
    ui->plot_1_3->addGraph(); // bright purple line
    ui->plot_1_3->graph(1)->setPen(linePen1);// goal pose z
    ui->plot_1_3->addGraph();
    ui->plot_1_3->graph(2)->setPen(linePen2); // pose setpoint z

    ui->plot_1_3->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_1_3->axisRect()->setupFullAxesBox(true);
    ui->plot_1_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_1_3->setRange(-100, 100);

    ui->plot_1_4->addGraph();
    ui->plot_1_4->graph(0)->setPen(bluePen);  // pose yaw
    ui->plot_1_4->addGraph();
    ui->plot_1_4->graph(1)->setPen(linePen1); // goal pose yaw
    ui->plot_1_4->addGraph();    
    ui->plot_1_4->graph(2)->setPen(linePen2); // pose setpoint yaw
    ui->plot_1_4->addGraph();    
    ui->plot_1_4->graph(3)->setPen(redPen);    // velocity_yaw yaw

    ui->plot_1_4->yAxis->setRange(-3.3, 3.3);
    //background lines
    ui->plot_1_4->axisRect()->setupFullAxesBox(true);
    ui->plot_1_4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_1_4->setRange(-100, 100);

//row 2
//velo 1234
//velo setpoint 1234
//velo_yaw 123
    ui->plot_2_1->addGraph(); // red line
    ui->plot_2_1->graph(0)->setPen(redPen); // velo x
    ui->plot_2_1->addGraph();
    ui->plot_2_1->graph(1)->setPen(linePen1); // velo setpoint x
    ui->plot_2_1->addGraph();
    ui->plot_2_1->graph(2)->setPen(linePen2); // velo_yaw x
    ui->plot_2_1->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_2_1->axisRect()->setupFullAxesBox(true);
    ui->plot_2_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_2_1->setRange(-100, 100);


    ui->plot_2_2->addGraph(); // green line
    ui->plot_2_2->graph(0)->setPen(greenPen); // velo y
    ui->plot_2_2->addGraph();
    ui->plot_2_2->graph(1)->setPen(linePen1); // velo setpoint y
    ui->plot_2_2->addGraph();
    ui->plot_2_2->graph(2)->setPen(linePen2); // velo_yaw y
    ui->plot_2_2->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_2_2->axisRect()->setupFullAxesBox(true);
    ui->plot_2_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_2_2->setRange(-100, 100);

    ui->plot_2_3->addGraph(); // blue line
    ui->plot_2_3->graph(0)->setPen(bluePen); // velo z
    ui->plot_2_3->addGraph();
    ui->plot_2_3->graph(1)->setPen(linePen1); // velo setpoint z
    ui->plot_2_3->addGraph();
    ui->plot_2_3->graph(2)->setPen(linePen2); // velo_yaw z
    ui->plot_2_3->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_2_3->axisRect()->setupFullAxesBox(true);
    ui->plot_2_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_2_3->setRange(-100, 100);

    ui->plot_2_4->addGraph();
    ui->plot_2_4->graph(0)->setPen(redPen); // velo yaw
    ui->plot_2_4->addGraph();
    ui->plot_2_4->graph(1)->setPen(linePen2); // velo setpoint yaw
    ui->plot_2_4->yAxis->setRange(-acos(-1.0), acos(-1.0));
    //background lines
    ui->plot_2_4->axisRect()->setupFullAxesBox(true);
    ui->plot_2_4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_2_4->setRange(-100, 100);

    if(plotting_row34 == 1)
    {
//row 3 acceleration
    ui->plot_3_1->addGraph(); // red line
    ui->plot_3_1->graph(0)->setPen(redPen);
    ui->plot_3_1->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_3_1->axisRect()->setupFullAxesBox(true);
    ui->plot_3_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_3_1->setRange(-100, 100);


    ui->plot_3_2->addGraph(); // green line
    ui->plot_3_2->graph(0)->setPen(greenPen);
    ui->plot_3_2->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_3_2->axisRect()->setupFullAxesBox(true);
    ui->plot_3_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_3_2->setRange(-100, 100);

    ui->plot_3_3->addGraph(); // blue line
    ui->plot_3_3->graph(0)->setPen(bluePen);
    ui->plot_3_3->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_3_3->axisRect()->setupFullAxesBox(true);
    ui->plot_3_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_3_3->setRange(-100, 100);

//row 4 jerk
    ui->plot_4_1->addGraph(); // red line
    ui->plot_4_1->graph(0)->setPen(redPen);
    ui->plot_4_1->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_4_1->axisRect()->setupFullAxesBox(true);
    ui->plot_4_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_4_1->setRange(-100, 100);


    ui->plot_4_2->addGraph(); // green line
    ui->plot_4_2->graph(0)->setPen(greenPen);
    ui->plot_4_2->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_4_2->axisRect()->setupFullAxesBox(true);
    ui->plot_4_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_4_2->setRange(-100, 100);

    ui->plot_4_3->addGraph(); // blue line
    ui->plot_4_3->graph(0)->setPen(bluePen);
    ui->plot_4_3->yAxis->setRange(-4, 4);
    //background lines
    ui->plot_4_3->axisRect()->setupFullAxesBox(true);
    ui->plot_4_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->verticalScrollBar_4_3->setRange(-100, 100);

    }// when plotting_row34 == 1

}




// position and orientation
void MainWindow::realtimeDataSlot1(double x,double y,double z,double roll,double pitch,double yaw)
{
//pos 
//goal 
//pose setpoint
//vel_yaw
//qDebug() << " plotting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
//###############################################################################
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    static double lastYaw = -100.0;

// for simplicity we just record offset(last time we click start button)
// and make the timeline back to 0
    double wanted_key = key-offset;
//qDebug() << "offset "<<offset;
    if (key-lastPointKey1 > 0.002) // at most add point every 2 ms
    {

      ui->plot_1_1->graph(0)->addData(wanted_key, x);
      ui->plot_1_2->graph(0)->addData(wanted_key, y);
      ui->plot_1_3->graph(0)->addData(wanted_key, z);

      if(fabs(lastYaw) > acos(-1.0) ) // need an initial value
          lastYaw = yaw;
      // yaw might have huge disturb on -pi to pi due to noise
      if( fabs( ( yaw - lastYaw )/(key-lastPointKey1) )>50.0 ) 
          yaw = lastYaw;

      ui->plot_1_4->graph(0)->addData(wanted_key, yaw);

      if(goal_val.size() > 0 )
      {
          ui->plot_1_1->graph(1)->addData(wanted_key, goal_val[0]);
          ui->plot_1_2->graph(1)->addData(wanted_key, goal_val[1]);
          ui->plot_1_3->graph(1)->addData(wanted_key, goal_val[2]);
          ui->plot_1_4->graph(1)->addData(wanted_key, goal_val[5]);
      }
      if(position_setpoint_val.size() > 0 )
      {
          ui->plot_1_1->graph(2)->addData(wanted_key, position_setpoint_val[0]);
          ui->plot_1_2->graph(2)->addData(wanted_key, position_setpoint_val[1]);
          ui->plot_1_3->graph(2)->addData(wanted_key, position_setpoint_val[2]);
          ui->plot_1_4->graph(2)->addData(wanted_key, position_setpoint_val[5]);
      }
      if(velocity_yaw_val.size() > 0 )
      {
          ui->plot_1_4->graph(3)->addData(wanted_key, velocity_yaw_val[3]);
      }

      ui->plot_1_1->graph(0)->rescaleValueAxis(true);
      ui->plot_1_2->graph(0)->rescaleValueAxis(true);
      ui->plot_1_3->graph(0)->rescaleValueAxis(true);
      ui->plot_1_4->graph(0)->rescaleValueAxis(true);
      
      ui->plot_1_1->graph(1)->rescaleValueAxis(true);
      ui->plot_1_2->graph(1)->rescaleValueAxis(true);
      ui->plot_1_3->graph(1)->rescaleValueAxis(true);
      ui->plot_1_4->graph(1)->rescaleValueAxis(true);

      ui->plot_1_1->graph(2)->rescaleValueAxis(true);
      ui->plot_1_2->graph(2)->rescaleValueAxis(true);
      ui->plot_1_3->graph(2)->rescaleValueAxis(true);
      ui->plot_1_4->graph(2)->rescaleValueAxis(true);

      ui->plot_1_4->graph(3)->rescaleValueAxis(true);

      lastYaw = yaw;
      lastPointKey1 = key;
    }

    if(stop == 0)
    {
        int lim = ui->horizontalScrollBar_1->maximum();
        //ui->horizontalScrollBar->setValue( int(round(key)) );
        ui->horizontalScrollBar_1->setValue( lim );
        
        // Range has to be int
        ui->horizontalScrollBar_1->setRange(0, int(ceil(wanted_key)));
        ui->plot_1_1->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
        ui->plot_1_2->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
        ui->plot_1_3->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
        ui->plot_1_4->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
    }
    else
    {
        ui->plot_1_1->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
        ui->plot_1_2->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
        ui->plot_1_3->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
        ui->plot_1_4->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
/*
        ui->plot_1_1->xAxis->setRange( ui->horizontalScrollBar_1->value() - offset, timescale_1, Qt::AlignRight);
        ui->plot_1_2->xAxis->setRange( ui->horizontalScrollBar_1->value() - offset, timescale_1, Qt::AlignRight);
        ui->plot_1_3->xAxis->setRange( ui->horizontalScrollBar_1->value() - offset, timescale_1, Qt::AlignRight);
        ui->plot_1_4->xAxis->setRange( ui->horizontalScrollBar_1->value() - offset, timescale_1, Qt::AlignRight);
*/
    }
//qDebug() << "lastPointKey "<<lastPointKey1;
//qDebug() << "timer "<< ui->horizontalScrollBar_1->value();
    ui->plot_1_1->replot();
    ui->plot_1_2->replot();
    ui->plot_1_3->replot();
    ui->plot_1_4->replot();
}


//velocity
void MainWindow::realtimeDataSlot2(double vx,double vy,double vz,double vroll,double vpitch,double vyaw)
{
//velo
//velo setpoint
//velo_yaw
//qDebug() << " plotting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
//###############################################################################
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;

// for simplicity we just record offset(last time we click start button)
// and make the timeline back to 0
    double wanted_key = key-offset;

    if (key-lastPointKey2 > 0.002) // at most add point every 2 ms
    {
      ui->plot_2_1->graph(0)->addData(wanted_key, vx);
      ui->plot_2_2->graph(0)->addData(wanted_key, vy);
      ui->plot_2_3->graph(0)->addData(wanted_key, vz);
      ui->plot_2_4->graph(0)->addData(wanted_key, vyaw);

      if(velocity_setpoint_val.size() > 0 )
      {
          ui->plot_2_1->graph(1)->addData(wanted_key, velocity_setpoint_val[0]);
          ui->plot_2_2->graph(1)->addData(wanted_key, velocity_setpoint_val[1]);
          ui->plot_2_3->graph(1)->addData(wanted_key, velocity_setpoint_val[2]);
          ui->plot_2_4->graph(1)->addData(wanted_key, velocity_setpoint_val[5]);
      }

      if(velocity_yaw_val.size() > 0 )
      {
          ui->plot_2_1->graph(2)->addData(wanted_key, velocity_yaw_val[0]);
          ui->plot_2_2->graph(2)->addData(wanted_key, velocity_yaw_val[1]);
          ui->plot_2_3->graph(2)->addData(wanted_key, velocity_yaw_val[2]);
      }

      ui->plot_2_1->graph(0)->rescaleValueAxis(true);
      ui->plot_2_2->graph(0)->rescaleValueAxis(true);
      ui->plot_2_3->graph(0)->rescaleValueAxis(true);
      ui->plot_2_4->graph(0)->rescaleValueAxis(true);      

      ui->plot_2_1->graph(1)->rescaleValueAxis(true);
      ui->plot_2_2->graph(1)->rescaleValueAxis(true);
      ui->plot_2_3->graph(1)->rescaleValueAxis(true);
      ui->plot_2_4->graph(1)->rescaleValueAxis(true);
      
      ui->plot_2_1->graph(2)->rescaleValueAxis(true);
      ui->plot_2_2->graph(2)->rescaleValueAxis(true);
      ui->plot_2_3->graph(2)->rescaleValueAxis(true);

      lastPointKey2 = key;
    }

    if(stop == 0)
    {
        int lim = ui->horizontalScrollBar_1->maximum();
        //ui->horizontalScrollBar->setValue( int(round(key)) );

        ui->horizontalScrollBar_1->setValue( lim );
        
        // Range has to be int
        ui->horizontalScrollBar_1->setRange(0, int(ceil(wanted_key)));
        ui->plot_2_1->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
        ui->plot_2_2->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
        ui->plot_2_3->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
        ui->plot_2_4->xAxis->setRange( wanted_key, timescale_1, Qt::AlignRight);
    }
    else
    {
        ui->plot_2_1->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
        ui->plot_2_2->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
        ui->plot_2_3->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
        ui->plot_2_4->xAxis->setRange( ui->horizontalScrollBar_1->value(), timescale_1, Qt::AlignRight);
    }
    ui->plot_2_1->replot();
    ui->plot_2_2->replot();
    ui->plot_2_3->replot();
    ui->plot_2_4->replot();
}


void MainWindow::realtimeDataSlot3(double ax,double ay,double az)
{
//qDebug() << " plotting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
//###############################################################################
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;

    static double last_ax = 0;
    static double last_ay = 0;
    static double last_az = 0;

// for simplicity we just record offset(last time we click start button)
// and make the timeline back to 0
    double period = key-lastPointKey;

    double wanted_key = key - offset;
    if (key-lastPointKey3 > 0.002) // at most add point every 2 ms
    {
      ui->plot_3_1->graph(0)->addData(wanted_key, ax);
      ui->plot_3_2->graph(0)->addData(wanted_key, ay);
      ui->plot_3_3->graph(0)->addData(wanted_key, az);

      ui->plot_3_1->graph(0)->rescaleValueAxis(true);
      ui->plot_3_2->graph(0)->rescaleValueAxis(true);
      ui->plot_3_3->graph(0)->rescaleValueAxis(true);
///*
      ui->plot_4_1->graph(0)->addData(wanted_key, (ax-last_ax)/period );
      ui->plot_4_2->graph(0)->addData(wanted_key, (ay-last_ay)/period );
      ui->plot_4_3->graph(0)->addData(wanted_key, (az-last_az)/period );

      ui->plot_4_1->graph(0)->rescaleValueAxis(true);
      ui->plot_4_2->graph(0)->rescaleValueAxis(true);
      ui->plot_4_3->graph(0)->rescaleValueAxis(true);
      lastPointKey3 = key;
      last_ax = ax;
      last_ay = ay;
      last_az = az;
//*/
    }

    if(stop == 0)
    {
        int lim = ui->horizontalScrollBar_2->maximum();

        ui->horizontalScrollBar_2->setValue( lim );
        
        // Range has to be int
        ui->horizontalScrollBar_2->setRange(0, int(ceil(wanted_key)));
        ui->plot_3_1->xAxis->setRange( wanted_key, timescale_2, Qt::AlignRight);
        ui->plot_3_2->xAxis->setRange( wanted_key, timescale_2, Qt::AlignRight);
        ui->plot_3_3->xAxis->setRange( wanted_key, timescale_2, Qt::AlignRight);
///*
        ui->plot_4_1->xAxis->setRange( wanted_key, timescale_2, Qt::AlignRight);
        ui->plot_4_2->xAxis->setRange( wanted_key, timescale_2, Qt::AlignRight);
        ui->plot_4_3->xAxis->setRange( wanted_key, timescale_2, Qt::AlignRight);
//*/

    }
    else
    {
        ui->plot_3_1->xAxis->setRange( ui->horizontalScrollBar_2->value(), timescale_2, Qt::AlignRight);
        ui->plot_3_2->xAxis->setRange( ui->horizontalScrollBar_2->value(), timescale_2, Qt::AlignRight);
        ui->plot_3_3->xAxis->setRange( ui->horizontalScrollBar_2->value(), timescale_2, Qt::AlignRight);
///*
        ui->plot_4_1->xAxis->setRange( ui->horizontalScrollBar_2->value(), timescale_2, Qt::AlignRight);
        ui->plot_4_2->xAxis->setRange( ui->horizontalScrollBar_2->value(), timescale_2, Qt::AlignRight);
        ui->plot_4_3->xAxis->setRange( ui->horizontalScrollBar_2->value(), timescale_2, Qt::AlignRight);
//*/
    }
    ui->plot_3_1->replot();
    ui->plot_3_2->replot();
    ui->plot_3_3->replot();
///*
    ui->plot_4_1->replot();
    ui->plot_4_2->replot();
    ui->plot_4_3->replot();
//*/
}
void MainWindow::setLegend()
{
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
//pos 1234
//goal 1234
//pose setpoint 1234
//vel_yaw 4

#define Alignment Qt::AlignTop|Qt::AlignRight
QColor legend_color = QColor(255,255,255,0);
    ui->plot_1_1->legend->setVisible(true);
    ui->plot_1_2->legend->setVisible(true);
    ui->plot_1_3->legend->setVisible(true);
    ui->plot_1_4->legend->setVisible(true);

    ui->plot_1_1->legend->setFont(legendFont);
    ui->plot_1_1->legend->setBrush(QBrush(legend_color));
    ui->plot_1_1->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_1_1->graph(0)->setName("x");
    ui->plot_1_1->graph(1)->setName("x goal");
    ui->plot_1_1->graph(2)->setName("x setpoint");
    ui->plot_1_1->xAxis->setLabel("time s");
    ui->plot_1_1->yAxis->setLabel("coordinate  m");

    ui->plot_1_2->legend->setFont(legendFont);
    ui->plot_1_2->legend->setBrush(QBrush(legend_color));
    ui->plot_1_2->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_1_2->graph(0)->setName("y");
    ui->plot_1_2->graph(1)->setName("y goal");
    ui->plot_1_2->graph(2)->setName("y setpoint");
    ui->plot_1_2->xAxis->setLabel("time s");
    ui->plot_1_2->yAxis->setLabel("coordinate  m");

    ui->plot_1_3->legend->setFont(legendFont);
    ui->plot_1_3->legend->setBrush(QBrush(legend_color));
    ui->plot_1_3->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_1_3->graph(0)->setName("z");
    ui->plot_1_3->graph(1)->setName("z goal");
    ui->plot_1_3->graph(2)->setName("z setpoint");
    ui->plot_1_3->xAxis->setLabel("time s");
    ui->plot_1_3->yAxis->setLabel("coordinate  m");

    ui->plot_1_4->legend->setFont(legendFont);
    ui->plot_1_4->legend->setBrush(QBrush(legend_color));
    ui->plot_1_4->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_1_4->graph(0)->setName("yaw");
    ui->plot_1_4->graph(1)->setName("yaw goal");
    ui->plot_1_4->graph(2)->setName("yaw setpoint");
    ui->plot_1_4->graph(3)->setName("yaw vel_yaw");
    ui->plot_1_4->xAxis->setLabel("time s");
    ui->plot_1_4->yAxis->setLabel("orientation rad");


//velo 1234
//velo setpoint 1234
//velo_yaw 123

    ui->plot_2_1->legend->setVisible(true);
    ui->plot_2_2->legend->setVisible(true);
    ui->plot_2_3->legend->setVisible(true);
    ui->plot_2_4->legend->setVisible(true);

    ui->plot_2_1->legend->setFont(legendFont);
    ui->plot_2_1->legend->setBrush(QBrush(legend_color));
    ui->plot_2_1->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_2_1->graph(0)->setName("vx");
    ui->plot_2_1->graph(1)->setName("vx setpoint");
    ui->plot_2_1->graph(2)->setName("vx vel_yaw");
    ui->plot_2_1->xAxis->setLabel("time s");
    ui->plot_2_1->yAxis->setLabel("velocity  m/s");

    ui->plot_2_2->legend->setFont(legendFont);
    ui->plot_2_2->legend->setBrush(QBrush(legend_color));
    ui->plot_2_2->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_2_2->graph(0)->setName("vy");
    ui->plot_2_2->graph(1)->setName("vy setpoint");
    ui->plot_2_2->graph(2)->setName("vy vel_yaw");
    ui->plot_2_2->xAxis->setLabel("time s");
    ui->plot_2_2->yAxis->setLabel("velocity  m/s");

    ui->plot_2_3->legend->setFont(legendFont);
    ui->plot_2_3->legend->setBrush(QBrush(legend_color));
    ui->plot_2_3->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_2_3->graph(0)->setName("vz");
    ui->plot_2_3->graph(1)->setName("vz setpoint");
    ui->plot_2_3->graph(2)->setName("vz vel_yaw");
    ui->plot_2_3->xAxis->setLabel("time s");
    ui->plot_2_3->yAxis->setLabel("velocity  m/s");

    ui->plot_2_4->legend->setFont(legendFont);
    ui->plot_2_4->legend->setBrush(QBrush(legend_color));
    ui->plot_2_4->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_2_4->graph(0)->setName("vyaw");
    ui->plot_2_4->graph(1)->setName("vyaw setpoint");
    ui->plot_2_4->xAxis->setLabel("time s");
    ui->plot_2_4->yAxis->setLabel("velocity  rad/s");

    if(plotting_row34 == 1)
    {
    ui->plot_3_1->legend->setVisible(true);
    ui->plot_3_2->legend->setVisible(true);
    ui->plot_3_3->legend->setVisible(true);

    ui->plot_3_1->legend->setFont(legendFont);
    ui->plot_3_1->legend->setBrush(QBrush(legend_color));
    ui->plot_3_1->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_3_1->graph(0)->setName("x accel");
    ui->plot_3_1->xAxis->setLabel("time s");
    ui->plot_3_1->yAxis->setLabel("accel  m/s^2");

    ui->plot_3_2->legend->setFont(legendFont);
    ui->plot_3_2->legend->setBrush(QBrush(legend_color));
    ui->plot_3_2->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_3_2->graph(0)->setName("y accel");
    ui->plot_3_2->xAxis->setLabel("time s");
    ui->plot_3_2->yAxis->setLabel("accel  m/s^2");

    ui->plot_3_3->legend->setFont(legendFont);
    ui->plot_3_3->legend->setBrush(QBrush(legend_color));
    ui->plot_3_3->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_3_3->graph(0)->setName("z accel");
    ui->plot_3_3->xAxis->setLabel("time s");
    ui->plot_3_3->yAxis->setLabel("accel  m/s^2");


    ui->plot_4_1->legend->setVisible(true);
    ui->plot_4_2->legend->setVisible(true);
    ui->plot_4_3->legend->setVisible(true);

    ui->plot_4_1->legend->setFont(legendFont);
    ui->plot_4_1->legend->setBrush(QBrush(legend_color));
    ui->plot_4_1->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_4_1->graph(0)->setName("x jerk");
    ui->plot_4_1->xAxis->setLabel("time s");
    ui->plot_4_1->yAxis->setLabel("jerk  m/s^3");

    ui->plot_4_2->legend->setFont(legendFont);
    ui->plot_4_2->legend->setBrush(QBrush(legend_color));
    ui->plot_4_2->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_4_2->graph(0)->setName("y jerk");
    ui->plot_4_2->xAxis->setLabel("time s");
    ui->plot_4_2->yAxis->setLabel("jerk  m/s^2");

    ui->plot_4_3->legend->setFont(legendFont);
    ui->plot_4_3->legend->setBrush(QBrush(legend_color));
    ui->plot_4_3->axisRect()->insetLayout()->setInsetAlignment(0, Alignment);
    ui->plot_4_3->graph(0)->setName("z jerk");
    ui->plot_4_3->xAxis->setLabel("time s");
    ui->plot_4_3->yAxis->setLabel("jerk  m/s^2");
    }// when plotting_row34 == 1

}
void MainWindow::TickerTime()
{
qDebug() << "new timer!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    QSharedPointer<QCPAxisTickerTime> newtimeTicker(new QCPAxisTickerTime) ;
    
    timeTicker = newtimeTicker;
    timeTicker->setTimeFormat("%s");
    ui->plot_1_1->xAxis->setTicker(timeTicker);
    ui->plot_1_2->xAxis->setTicker(timeTicker);
    ui->plot_1_3->xAxis->setTicker(timeTicker);
    ui->plot_1_4->xAxis->setTicker(timeTicker);

    ui->plot_2_1->xAxis->setTicker(timeTicker);
    ui->plot_2_2->xAxis->setTicker(timeTicker);
    ui->plot_2_3->xAxis->setTicker(timeTicker);
    ui->plot_2_4->xAxis->setTicker(timeTicker);
    
    if(plotting_row34 == 1)
    {
    ui->plot_3_1->xAxis->setTicker(timeTicker);
    ui->plot_3_2->xAxis->setTicker(timeTicker);
    ui->plot_3_3->xAxis->setTicker(timeTicker);
    
    ui->plot_4_1->xAxis->setTicker(timeTicker);
    ui->plot_4_2->xAxis->setTicker(timeTicker);
    ui->plot_4_3->xAxis->setTicker(timeTicker);
    }//when plotting_row34 == 1
}


void MainWindow::connections()
{
qDebug() << "connections!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this, SLOT(slot_rosShutdown()));

    //connect(ui->plot_1_1->xAxis, SIGNAL( rangeChanged(QCPRange) ), this, SLOT(xAxis_1_Changed( std::pair<int, QCPRange> ) )) ;
    //connect(ui->plot_1_1->xAxis, SIGNAL(p1(1,rangeChanged(QCPRange)) ), this, SLOT(xAxisChanged( std::pair<int, QCPRange> ) )) ;

    connect(ui->plot_1_1->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    connect(ui->plot_1_2->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    connect(ui->plot_1_3->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    connect(ui->plot_1_4->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;

    connect(ui->plot_2_1->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    connect(ui->plot_2_2->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    connect(ui->plot_2_3->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    connect(ui->plot_2_4->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_1_Changed(QCPRange) )) ;
    if(plotting_row34 == 1)
    {
    connect(ui->plot_3_1->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_2_Changed(QCPRange) )) ;
    connect(ui->plot_3_2->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_2_Changed(QCPRange) )) ;
    connect(ui->plot_3_3->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_2_Changed(QCPRange) )) ;

    connect(ui->plot_4_1->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_2_Changed(QCPRange) )) ;
    connect(ui->plot_4_2->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_2_Changed(QCPRange) )) ;
    connect(ui->plot_4_3->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxis_2_Changed(QCPRange) )) ;
    }//when plotting_row34 == 1

    connect(ui->plot_1_1->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_1_1_Changed(QCPRange)));
    connect(ui->plot_1_2->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_1_2_Changed(QCPRange)));
    connect(ui->plot_1_3->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_1_3_Changed(QCPRange)));
    connect(ui->plot_1_4->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_1_4_Changed(QCPRange)));

    connect(ui->plot_2_1->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_2_1_Changed(QCPRange)));
    connect(ui->plot_2_2->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_2_2_Changed(QCPRange)));
    connect(ui->plot_2_3->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_2_3_Changed(QCPRange)));
    connect(ui->plot_2_4->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_2_4_Changed(QCPRange)));

    if(plotting_row34 == 1)
    {
    connect(ui->plot_3_1->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_3_1_Changed(QCPRange)));
    connect(ui->plot_3_2->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_3_2_Changed(QCPRange)));
    connect(ui->plot_3_3->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_3_3_Changed(QCPRange)));
    connect(ui->plot_4_1->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_4_1_Changed(QCPRange)));
    connect(ui->plot_4_2->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_4_2_Changed(QCPRange)));
    connect(ui->plot_4_3->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxis_4_3_Changed(QCPRange)));
    }//when plotting_row34 == 1

    connect(ui->verticalScrollBar_1_1, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_1_1_Changed(int)));
    connect(ui->verticalScrollBar_1_2, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_1_2_Changed(int)));
    connect(ui->verticalScrollBar_1_3, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_1_3_Changed(int)));
    connect(ui->verticalScrollBar_1_4, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_1_4_Changed(int)));

    connect(ui->verticalScrollBar_2_1, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_2_1_Changed(int)));
    connect(ui->verticalScrollBar_2_2, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_2_2_Changed(int)));
    connect(ui->verticalScrollBar_2_3, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_2_3_Changed(int)));
    connect(ui->verticalScrollBar_2_4, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_2_4_Changed(int)));

    if(plotting_row34 == 1)
    {
    connect(ui->verticalScrollBar_3_1, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_3_1_Changed(int)));
    connect(ui->verticalScrollBar_3_2, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_3_2_Changed(int)));
    connect(ui->verticalScrollBar_3_3, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_3_3_Changed(int)));
    connect(ui->verticalScrollBar_4_1, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_4_1_Changed(int)));
    connect(ui->verticalScrollBar_4_2, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_4_2_Changed(int)));
    connect(ui->verticalScrollBar_4_3, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBar_4_3_Changed(int)));
    }//when plotting_row34 == 1

    connect(ui->timescaleSlider_1, SIGNAL(valueChanged(int)) , this , SLOT( timescaleSlider_1_Changed()));
    connect(ui->timescaleSlider_2, SIGNAL(valueChanged(int)) , this , SLOT( timescaleSlider_2_Changed()));

    connect(ui->gripperSlider, SIGNAL(valueChanged(int)) , this , SLOT( gripperSliderChanged()));


    connect(ui-> save_pic_btn , SIGNAL(clicked()) , this , SLOT( save_pic_btn_clicked() ) );
    connect(ui-> update_btn , SIGNAL(clicked()) , this , SLOT( update_btn_clicked() ) );
    connect(ui-> pause_btn , SIGNAL(clicked()) , this , SLOT( pause_btn_clicked() ) );

    connect(ui-> gripper_open_btn , SIGNAL(clicked()) , this , SLOT( gripper_open_btn_clicked() ) );
    connect(ui-> gripper_close_btn , SIGNAL(clicked()) , this , SLOT( gripper_open_btn_clicked() ) );


    connect(ui->start_recording_btn,SIGNAL(clicked()),this,SLOT(start_recording() ) );
    connect(ui->stop_recording_btn,SIGNAL(clicked()),this,SLOT(stop_recording() ) );

    connect(ui->start_btn,SIGNAL(clicked()),this,SLOT(start_from_the_beginning()) );
    //connect(ui->start_btn,SIGNAL(clicked()),this,SLOT(TickerTime()) );
    
    connect(ui->refreash_topic_btn,SIGNAL(clicked()),this,SLOT(refreashTopicList() ) );

    // power signal emit from qnode

    connect(&qnode,SIGNAL(power(float,float)),this,SLOT(slot_power(float,float)));
    connect(&qnode,SIGNAL(state(bool,std::string)),this,SLOT(slot_state(bool,std::string)));
    connect(&qnode,SIGNAL(error_info(std::string)) , this , SLOT( slot_error_info(std::string) ) );

    connect(&qnode,SIGNAL(gripper(int)),this,SLOT(slot_gripper(int)));

    connect(&qnode,SIGNAL(position(double,double,double,double,double,double)),this,SLOT(realtimeDataSlot1(double,double,double,double,double,double)));
    connect(&qnode,SIGNAL(velocity(double,double,double,double,double,double)),this,SLOT(realtimeDataSlot2(double,double,double,double,double,double)));

    if(plotting_row34 == 1)
    {
    connect(&qnode,SIGNAL(accel(double,double,double)),this,SLOT(realtimeDataSlot3(double,double,double)));
    }//when plotting_row34 == 1

    connect(&qnode,SIGNAL(position(double,double,double,double,double,double)),this,SLOT(slot_position(double,double,double,double,double,double)));
    connect(&qnode,SIGNAL(goal(double,double,double,double,double,double)),this,SLOT(slot_goal(double,double,double,double,double,double)));

    connect(&qnode,SIGNAL(velocity_yaw(double,double,double,double)),this,SLOT(slot_velocity_yaw(double,double,double,double)));
    connect(&qnode,SIGNAL(velocity_setpoint(double,double,double,double,double,double)),this,SLOT(slot_velocity_setpoint(double,double,double,double,double,double)));
    
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(ROSRUN() ));
    timer->start(1);
} // end of connections()

void MainWindow::disconnections()
{
qDebug() << "disconnections!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    
    (&qnode)->disconnect();

    //connect(ui->plot_1_1->xAxis, SIGNAL( rangeChanged(QCPRange) ), this, SLOT(xAxis_1_Changed( std::pair<int, QCPRange> ) )) ;
    //connect(ui->plot_1_1->xAxis, SIGNAL(p1(1,rangeChanged(QCPRange)) ), this, SLOT(xAxisChanged( std::pair<int, QCPRange> ) )) ;

    ui->plot_1_1->xAxis->disconnect();
    ui->plot_1_2->xAxis->disconnect();
    ui->plot_1_3->xAxis->disconnect();
    ui->plot_1_4->xAxis->disconnect();

    ui->plot_2_1->xAxis->disconnect();
    ui->plot_2_2->xAxis->disconnect();
    ui->plot_2_3->xAxis->disconnect();
    ui->plot_2_4->xAxis->disconnect();

    ui->plot_3_1->xAxis->disconnect();
    ui->plot_3_2->xAxis->disconnect();
    ui->plot_3_3->xAxis->disconnect();

    ui->plot_4_1->xAxis->disconnect();
    ui->plot_4_2->xAxis->disconnect();
    ui->plot_4_3->xAxis->disconnect();

    ui->plot_1_1->yAxis->disconnect();
    ui->plot_1_2->yAxis->disconnect();
    ui->plot_1_3->yAxis->disconnect();
    ui->plot_1_4->yAxis->disconnect();

    ui->plot_2_1->yAxis->disconnect();
    ui->plot_2_2->yAxis->disconnect();
    ui->plot_2_3->yAxis->disconnect();
    ui->plot_2_4->yAxis->disconnect();

    ui->plot_3_1->yAxis->disconnect();
    ui->plot_3_2->yAxis->disconnect();
    ui->plot_3_3->yAxis->disconnect();

    ui->plot_4_1->yAxis->disconnect();
    ui->plot_4_2->yAxis->disconnect();
    ui->plot_4_3->yAxis->disconnect();

    ui->verticalScrollBar_1_1->disconnect();
    ui->verticalScrollBar_1_2->disconnect();
    ui->verticalScrollBar_1_3->disconnect();
    ui->verticalScrollBar_1_4->disconnect();

    ui->verticalScrollBar_2_1->disconnect();
    ui->verticalScrollBar_2_2->disconnect();
    ui->verticalScrollBar_2_3->disconnect();
    ui->verticalScrollBar_2_4->disconnect();

    ui->verticalScrollBar_3_1->disconnect();
    ui->verticalScrollBar_3_2->disconnect();
    ui->verticalScrollBar_3_3->disconnect();
    ui->verticalScrollBar_4_1->disconnect();
    ui->verticalScrollBar_4_2->disconnect();
    ui->verticalScrollBar_4_3->disconnect();

    ui->timescaleSlider_1->disconnect();
    ui->timescaleSlider_2->disconnect();

    ui->gripperSlider->disconnect();


    ui-> save_pic_btn->disconnect();
    ui-> update_btn->disconnect(); 
    ui-> pause_btn->disconnect();

    ui-> gripper_open_btn->disconnect();
    ui-> gripper_close_btn->disconnect();


    ui->start_recording_btn->disconnect();
    ui->stop_recording_btn->disconnect();

    ui->start_btn->disconnect();

    ui->refreash_topic_btn->disconnect();

    timer->disconnect();
} // end of disconnections()



void MainWindow::slot_state(bool armed,std::string mode)
{
    if(armed)
    {
        ui->label_staue->setStyleSheet("color:green;");
        ui->label_staue->setText("armed");
    }
    else
    {
        ui->label_staue->setStyleSheet("color:red;");
        ui->label_staue->setText("unarmed");
    }

    if(mode == "OFFBOARD")
    	ui->label_mode->setStyleSheet("color:green;");
    else
        ui->label_mode->setStyleSheet("color:blue;");

    ui->label_mode->setText(QString::fromStdString(mode));

}

void MainWindow::slot_error_info(std::string info)
{
    ui->statusbar->showMessage(QString::fromStdString(info));
    ui->statusbar->setStyleSheet("color:red;");
}

void MainWindow::horzScrollBar_1_Changed(int value)
{

}
void MainWindow::horzScrollBar_2_Changed(int value)
{
  
}
void MainWindow::timescaleSlider_1_Changed()
{
     timescale_1 = ui->timescaleSlider_1->value();
     ui->timescaleLabel_1->setText( QString::number(timescale_1) );
}

void MainWindow::timescaleSlider_2_Changed()
{
     timescale_2 = ui->timescaleSlider_2->value();
     ui->timescaleLabel_2->setText( QString::number(timescale_2) );
}

void MainWindow::gripperSliderChanged()
{
     int degree = ui->gripperSlider->value();
     ui->gripper_degree_label->setText( QString::number(degree) );
}
/*
void MainWindow::vertScrollBar_1_1_Changed(QCustomPlot *customPlot , int value )
{
  customPlot->yAxis->setRange(-value/100.0, ui->plot_1_1->yAxis->range().size(), Qt::AlignCenter);
  customPlot->replot();
}
*/

//void MainWindow::vertScrollBarChanged(int value)

void MainWindow::vertScrollBar_1_1_Changed(int value)
{
  //if (qAbs(ui->plot_1_1->yAxis->range().center()-value/100.0) > 0.01) // if user is dragging plot, we don't want to replot twice
  
    ui->plot_1_1->yAxis->setRange(-value/100.0, ui->plot_1_1->yAxis->range().size(), Qt::AlignCenter);
    ui->plot_1_1->replot();
  
}
void MainWindow::vertScrollBar_1_2_Changed(int value)
{
  ui->plot_1_2->yAxis->setRange(-value/100.0, ui->plot_1_2->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_1_2->replot();
}
void MainWindow::vertScrollBar_1_3_Changed(int value)
{
  ui->plot_1_3->yAxis->setRange(-value/100.0, ui->plot_1_3->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_1_3->replot();
}
void MainWindow::vertScrollBar_1_4_Changed(int value)
{
  ui->plot_1_4->yAxis->setRange(-value/100.0, ui->plot_1_4->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_1_4->replot();
}
void MainWindow::vertScrollBar_2_1_Changed(int value)
{
  ui->plot_2_1->yAxis->setRange(-value/100.0, ui->plot_2_1->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_2_1->replot();
}
void MainWindow::vertScrollBar_2_2_Changed(int value)
{
  ui->plot_2_2->yAxis->setRange(-value/100.0, ui->plot_2_2->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_2_2->replot();
}
void MainWindow::vertScrollBar_2_3_Changed(int value)
{
  ui->plot_2_3->yAxis->setRange(-value/100.0, ui->plot_2_3->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_2_3->replot();
}
void MainWindow::vertScrollBar_2_4_Changed(int value)
{
  ui->plot_2_4->yAxis->setRange(-value/100.0, ui->plot_2_4->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_2_4->replot();
}
void MainWindow::vertScrollBar_3_1_Changed(int value)
{
  ui->plot_3_1->yAxis->setRange(-value/100.0, ui->plot_3_1->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_3_1->replot();
}
void MainWindow::vertScrollBar_3_2_Changed(int value)
{
  ui->plot_3_2->yAxis->setRange(-value/100.0, ui->plot_3_2->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_3_2->replot();
}
void MainWindow::vertScrollBar_3_3_Changed(int value)
{
  ui->plot_3_3->yAxis->setRange(-value/100.0, ui->plot_3_3->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_3_3->replot();
}
void MainWindow::vertScrollBar_4_1_Changed(int value)
{
  ui->plot_4_1->yAxis->setRange(-value/100.0, ui->plot_4_1->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_4_1->replot();
}
void MainWindow::vertScrollBar_4_2_Changed(int value)
{
  ui->plot_4_2->yAxis->setRange(-value/100.0, ui->plot_4_2->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_4_2->replot();
}
void MainWindow::vertScrollBar_4_3_Changed(int value)
{
  ui->plot_4_3->yAxis->setRange(-value/100.0, ui->plot_4_3->yAxis->range().size(), Qt::AlignCenter);
  ui->plot_4_3->replot();
}


void MainWindow::slot_gripper(int command)
{
    ui->gripper_label->setText( QString::number(command));
}

//update current position
void MainWindow::slot_position(double x,double y,double z,double roll,double pitch,double yaw)
{
    ui->pose_x_label->setText( QString::number(x).mid(0,4) );
    ui->pose_y_label->setText( QString::number(y).mid(0,4) );
    ui->pose_z_label->setText( QString::number(z).mid(0,4) );
// this max_height is defined on the top of the code, in case you need to change it
    if( z > max_height )
    {
// when I define a public variable called_max_height in MainWindow, the code crushes, IDK Y
//qDebug() << "entered if !!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        ui->pose_z_label->setStyleSheet("color:red;");
    }

    ui->pose_roll_label->setText( QString::number(roll).mid(0,4) );
    ui->pose_pitch_label->setText( QString::number(pitch).mid(0,4) );
    ui->pose_yaw_label->setText( QString::number(yaw).mid(0,4) );


}
//update goal position
void MainWindow::slot_goal(double x,double y,double z,double roll,double pitch,double yaw)
{
qDebug() << "got a goal !!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    ui->goal_x_label->setText( QString::number(x).mid(0,4) );
    ui->goal_y_label->setText( QString::number(y).mid(0,4) );
    ui->goal_z_label->setText( QString::number(z).mid(0,4) );
    ui->goal_roll_label->setText( QString::number(roll).mid(0,4) );
    ui->goal_pitch_label->setText( QString::number(pitch).mid(0,4) );
    ui->goal_yaw_label->setText( QString::number(yaw).mid(0,4) );

    std::vector<double> val;
    val.push_back(x); val.push_back(y); val.push_back(z);
    val.push_back(roll); val.push_back(pitch); val.push_back(yaw);
    goal_val = val;
}
void MainWindow::slot_position_setpoint(double x,double y,double z,double roll,double pitch,double yaw)
{
qDebug() << "got a position_setpoint !!!!!!!!!!!!!!!!";
    std::vector<double> val;
    val.push_back(x); val.push_back(y); val.push_back(z);
    val.push_back(roll); val.push_back(pitch); val.push_back(yaw);
    position_setpoint_val = val;
    int flag = 0;
    if(position_setpoint_val.size() != goal_val.size() ) flag = 1 ;
    else
    {
        for(int i=0 ; i<goal_val.size() ; i++ )
        {
            if( abs( goal_val[i] - position_setpoint_val[i] ) > 1e-5 ) flag = 1;  
        }
    }

    if(flag == 1)
    { 
        ui->pose_setpoint_label->setText("SAME");
        ui->pose_setpoint_label->setStyleSheet("color:green;");
    }
    else
    { 
        ui->pose_setpoint_label->setText("NOT THE SAME");
        ui->pose_setpoint_label->setStyleSheet("color:red;");
    }

}
void MainWindow::slot_velocity_setpoint(double x,double y,double z,double roll,double pitch,double yaw)
{
//qDebug() << "got a velocity_setpoint !!!!!!!!!!!!!!!!";
    std::vector<double> val;
    val.push_back(x); val.push_back(y); val.push_back(z);
    val.push_back(roll); val.push_back(pitch); val.push_back(yaw);
    velocity_setpoint_val = val;
    ui->velocity_setpoint_vx_label->setText( QString::number(x).mid(0,4) );
    ui->velocity_setpoint_vy_label->setText( QString::number(y).mid(0,4) );
    ui->velocity_setpoint_vz_label->setText( QString::number(z).mid(0,4) );
    ui->velocity_setpoint_vyaw_label->setText( QString::number(yaw).mid(0,4) );
}

void MainWindow::slot_velocity_yaw(double x,double y,double z,double yaw)
{
qDebug() << "got a velocity_yaw !!!!!!!!!!!!!!!!!!!!!";
    std::vector<double> val;
    val.push_back(x); val.push_back(y); val.push_back(z); val.push_back(yaw);
    velocity_yaw_val = val;
    ui->velocity_yaw_vx_label->setText( QString::number(x).mid(0,4) );
    ui->velocity_yaw_vy_label->setText( QString::number(y).mid(0,4) );
    ui->velocity_yaw_vz_label->setText( QString::number(z).mid(0,4) );
    ui->velocity_yaw_yaw_label->setText( QString::number(yaw).mid(0,4) );
}
//////////////////////////////////////////
///to be continued
//////////////////////////////////////////
void MainWindow::gripper_open_btn_clicked()
{
    qnode.gripper_publish(0);
}

void MainWindow::gripper_close_btn_clicked()
{
    qnode.gripper_publish(int( ui->gripperSlider->value() ));
}
void MainWindow::save_pic_btn_clicked()
{ 
/*
    int check;
    char dirname[20] = "saved_plots";
  
    check = mkdir(dirname,0777);
  
    // check if directory is created or not
    if (!check)
        qDebug() <<"Directory created\n";
    else 
    {
        qDebug() <<"Unable to create directory\n";
    }
*/


    std::string Path = "./saved_plots/test_" + std::to_string( int(ui->test_num->value()) ) ;
    
    std::string str = "mkdir -p " + Path;
    char cmd[str.size() + 1];
    strcpy( cmd , str.c_str() );
    
    //make an independent direction for each test
    DIR *dir;   
    if ((dir=opendir(Path.c_str())) == NULL)   
    {   
        //system("mkdir -p ./saved_plots");      //system( s.c_str() );
	system(cmd);
    }  

    QString time_str;
    QTime time(QTime::currentTime());
    time_str = time.toString("hh-mm-ss-");
qDebug()<<"plots saved at: " + time_str;
//qDebug()<<time;
    
/*
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) qDebug() << file.errorString();
    else {}
*/

    QString test_number =  QString::number(int(ui->test_num->value()));
    QString prefix = QString::fromStdString(Path) + "/" + "test_" + test_number + "_" + time_str;
    
    QString filename;

    filename = prefix + "position_x.png";
    ui->plot_1_1->savePng( filename,ui->plot_1_1->width(), ui->plot_1_1->height() );
    filename = prefix + "position_y.png";
    ui->plot_1_2->savePng( filename,ui->plot_1_2->width(), ui->plot_1_2->height() );
    filename = prefix + "position_z.png";
    ui->plot_1_3->savePng( filename,ui->plot_1_3->width(), ui->plot_1_3->height() );
    filename = prefix + "position_yaw.png";
    ui->plot_1_4->savePng( filename,ui->plot_1_4->width(), ui->plot_1_4->height() );

    filename = prefix + "velocity_x.png";
    ui->plot_2_1->savePng( filename,ui->plot_2_1->width(), ui->plot_2_1->height() );
    filename = prefix + "velocity_y.png";
    ui->plot_2_2->savePng( filename,ui->plot_2_2->width(), ui->plot_2_2->height() );
    filename = prefix + "velocity_z.png";
    ui->plot_2_3->savePng( filename,ui->plot_2_3->width(), ui->plot_2_3->height() );
    filename = prefix + "velocity_yaw.png";
    ui->plot_2_4->savePng( filename,ui->plot_2_4->width(), ui->plot_2_4->height() );

}




void MainWindow::xAxis_1_Changed(QCPRange range)
{
  //qDebug() << "xAxisChanged";
    timescale_1 = qRound(range.size());
    ui->timescaleLabel_1->setText( QString::number(timescale_1) );
    ui->timescaleSlider_1->setValue(timescale_1);
    ui->horizontalScrollBar_1->setPageStep(timescale_1);
}
void MainWindow::xAxis_2_Changed(QCPRange range)
{
    timescale_2 = qRound(range.size());
    ui->timescaleLabel_2->setText( QString::number(timescale_2) );
    ui->timescaleSlider_2->setValue(timescale_2);
    ui->horizontalScrollBar_2->setPageStep(timescale_2);
}


//void MainWindow::yAxisChanged(QScrollBar *verticalScrollBar,QCPRange range)
void MainWindow::yAxis_1_1_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_1_1->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_1_1->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_1_2_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_1_2->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_1_2->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_1_3_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_1_3->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_1_3->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_1_4_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_1_4->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_1_4->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_2_1_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_2_1->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_2_1->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_2_2_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_2_2->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_2_2->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_2_3_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_2_3->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_2_3->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_2_4_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_2_4->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_2_4->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}


void MainWindow::yAxis_3_1_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_3_1->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_3_1->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_3_2_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_3_2->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_3_2->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_3_3_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_3_3->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_3_3->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_4_1_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_4_1->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_4_1->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_4_2_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_4_2->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_4_2->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}
void MainWindow::yAxis_4_3_Changed(QCPRange range)
{
//  qDebug() << "yAxisChanged";
//qDebug() << range.size();
  ui->verticalScrollBar_4_3->setValue(qRound(-range.center()*100.0)); // adjust position of scroll bar slider
  ui->verticalScrollBar_4_3->setPageStep(qRound(range.size()*100.0)); // adjust size of scroll bar slider
}


void MainWindow::update_btn_clicked()
{
   stop = 0;
}
void MainWindow::pause_btn_clicked()
{
   stop = 1;
}

void MainWindow::start_recording()
{

// change the index
    record_num++;
    std::string filename = "record" + std::to_string(record_num) + ".bag";
    std::string node_name = " __name:=my_record_node";
    stop_recording();

///////////////////////////////////////////////////////////
// directly use cmd to do recording is the easiest way
// might need to appoint specific topics in future version
///////////////////////////////////////////////////////////

//commander line for recording multiple topics
//rosbag record -O filename(*.bag) /topic1 /topic2 /topic2 

    //std::string cmd_str = "gnome-terminal --geometry=0x0+10+10 -- bash -c 'rosbag record -a -O "+ filename + node_name + "'";

// start and close recording have to be in 2 different terminals       
//std::string cmd_str = "rosbag record -a -O "+ filename + node_name ;
//"gnome-terminal -- bash -c 'rosbag record -a -O record2.bag' "
    

// execute

std::string cmd_str = "gnome-terminal --geometry=0x0+10+10 -- bash -c 'source $(find )/devel/setup.bash & roslaunch uav_simulator bag.launch' ";
//std::string cmd_str = "roslaunch uav_simulator bag.launch";
    int ret = system(cmd_str.c_str());

}
void MainWindow::stop_recording()
{
	ros::V_string v_nodes;
	ros::master::getNodes(v_nodes);

	std::string node_name = std::string("/rosbag_record");
	auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name.c_str());
	if (it != v_nodes.end()){
	    std::string cmd_str = "rosnode kill " + node_name;
	    int ret = system(cmd_str.c_str());
	    std::cout << "## stop rosbag record cmd: " << cmd_str << std::endl;
	}
// stop updating
	stop = 1;
}



//////////////////////////////////////////////////////
// some messages
//////////////////////////////////////////////////////
void MainWindow::inform(QString strdata)
{
    QMessageBox m_r;
    m_r.setWindowTitle("notification");
    m_r.setText(strdata);
    m_r.exec();
}

void MainWindow::initTopicList()
{
qDebug() << "topics!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    ui->topic_listWidget->clear();
    ui->topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
       ui->topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
}
void MainWindow::refreashTopicList()
{
    initTopicList();
}

void MainWindow::slot_rosShutdown()
{
    ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
    ui->label_statue_text->setStyleSheet("color:red;");
    ui->label_statue_text->setText("offline");

}
void MainWindow::slot_power(float percentage , float voltage)
{
    ui->label_power->setText(QString::number(static_cast<double>(voltage)).mid(0,4)+"V");
    int value = static_cast<int>(percentage*100);
    ui->progressBar->setValue(value>100?100:value);
    ui->progressBar->setStyleSheet("QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
}

void MainWindow::ReadSettings() {
}

void MainWindow::WriteSettings() {
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

} // end of namespace px4_gui


