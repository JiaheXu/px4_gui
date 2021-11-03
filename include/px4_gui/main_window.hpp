
#ifndef px4_gui_MAIN_WINDOW_H
#define px4_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QSoundEffect>
#include <QComboBox>
#include <QSpinBox>
#include <QTimer>
#include <vector>
#include <QVariant>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>
#include <QFileDialog>
#include <map>


namespace Ui {
class MainWindow;
}

/*****************************************************************************
** Namespace
*****************************************************************************/
namespace px4_gui {


class MainWindow : public QMainWindow
{




    Q_OBJECT
    
    public:
      int plotting_row34 = 0;
      int timescale_1=30;
      int timescale_2=30;
      int stop = 0;
      int stop_1 = 0;
      int stop_2 = 0;
      int record_num=0;
      double lastPointKey1=0;
      double lastPointKey2=0;
      double lastPointKey3=0;
      double offset = 0;

      QSharedPointer<QCPAxisTickerTime> timeTicker;

      std::vector<double> position_setpoint_val;
      std::vector<double> velocity_setpoint_val;
      std::vector<double> goal_val;
      std::vector<double> velocity_yaw_val;

      MainWindow(int argc, char** argv, QWidget *parent = nullptr);
      ~MainWindow();
    
      void ReadSettings(); // Load up qt program settings at startup
      void WriteSettings(); // Save qt program settings when closing
    
      void closeEvent(QCloseEvent *event); // Overloaded function
      
      
      void setLegend();
      void initQcustomplot();
      void initTopicList();
      
      void connections();
      void disconnections();
    public slots:
        void TickerTime();
        void start_from_the_beginning();
        void start_recording();
        void stop_recording();
        void pause_btn_clicked();
        void update_btn_clicked();
        void save_pic_btn_clicked();
        void gripper_open_btn_clicked();
        void gripper_close_btn_clicked();

        void slot_rosShutdown();
        void refreashTopicList();

        void slot_power(float p,float v);
        void slot_state(bool, std::string);
        void slot_error_info(std::string);
        void slot_position(double,double,double,double,double,double);

        void slot_goal(double,double,double,double,double,double);
        void slot_position_setpoint(double,double,double,double,double,double);
        void slot_velocity_setpoint(double,double,double,double,double,double);
        void slot_velocity_yaw(double,double,double,double);
        void slot_gripper(int);
 
        void vertScrollBar_1_1_Changed(int value);
        void vertScrollBar_1_2_Changed(int value);
        void vertScrollBar_1_3_Changed(int value);
        void vertScrollBar_1_4_Changed(int value);
        void vertScrollBar_2_1_Changed(int value);
        void vertScrollBar_2_2_Changed(int value);
        void vertScrollBar_2_3_Changed(int value);
        void vertScrollBar_2_4_Changed(int value);
        void vertScrollBar_3_1_Changed(int value);
        void vertScrollBar_3_2_Changed(int value);
        void vertScrollBar_3_3_Changed(int value);
        void vertScrollBar_4_1_Changed(int value);
        void vertScrollBar_4_2_Changed(int value);
        void vertScrollBar_4_3_Changed(int value);


        //void vertScrollBar_1_1_Changed(QCustomPlot *customPlot,int value);
        //void horzScrollBarChanged(QCustomPlot *customPlot,int value);

        //void vertScrollBarChanged(int value);
        void horzScrollBar_1_Changed(int value);
        void horzScrollBar_2_Changed(int value);

        void timescaleSlider_1_Changed();
        void timescaleSlider_2_Changed();
        void gripperSliderChanged();

        //void xAxisChanged(QCustomPlot *customPlot , QCPRange);
        //void yAxisChanged(QCustomPlot *customPlot , QCPRange);
        void xAxis_1_Changed(QCPRange);
        void xAxis_2_Changed(QCPRange);
        //void xAxisChanged( std::pair<int, QCPRange> );
        //void xAxisChanged( QCPRange);

        void yAxis_1_1_Changed( QCPRange);
        void yAxis_1_2_Changed( QCPRange);
        void yAxis_1_3_Changed( QCPRange);
        void yAxis_1_4_Changed( QCPRange);

        void yAxis_2_1_Changed( QCPRange);
        void yAxis_2_2_Changed( QCPRange);
        void yAxis_2_3_Changed( QCPRange);
        void yAxis_2_4_Changed( QCPRange);

        void yAxis_3_1_Changed( QCPRange);
        void yAxis_3_2_Changed( QCPRange);
        void yAxis_3_3_Changed( QCPRange);

        void yAxis_4_1_Changed( QCPRange);
        void yAxis_4_2_Changed( QCPRange);
        void yAxis_4_3_Changed( QCPRange);
        //void yAxisChanged(QScrollBar *verticalScrollBar,QCPRange);

        void ROSRUN(); 

        void realtimeDataSlot1(double x,double y,double z,double roll,double pitch,double yaw);
        void realtimeDataSlot2(double x,double y,double z,double roll,double pitch,double yaw);
        void realtimeDataSlot3(double x,double y,double z);
    private:
        Ui::MainWindowDesign *ui;
      
        
        void inform(QString);
        
        QNode qnode;
        QTimer* timer;


};
}// namespace px4_gui

#endif // px4_gui_MAIN_WINDOW_H
