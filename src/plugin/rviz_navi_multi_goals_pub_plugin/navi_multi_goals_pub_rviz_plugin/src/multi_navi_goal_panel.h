#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H


#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QComboBox>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>


namespace navi_multi_goals_pub_rviz_plugin {



    class MultiNaviGoalsPanel : public rviz::Panel {
    Q_OBJECT
    public:
        explicit MultiNaviGoalsPanel(QWidget *parent = 0);

    public Q_SLOTS:

        void setMaxNumGoal(const QString &maxNumGoal);

        void writePose(geometry_msgs::Pose pose);
        void markPose(const geometry_msgs::PoseStamped::ConstPtr &pose,int id);
        void deleteMark();
        void RemarkPose(geometry_msgs::PoseStamped &pose,int id);
        std::string getDirPath();

    protected Q_SLOTS:

        void startMapping();                  //launch mapping node
        void stopMapping();                  //stop mapping node
        void recordMapping();                //start record map
        void initDialog();
        void saveFileBtnSlot();
        void mapTableSlot(int index);
        void clearMap();

        void updateMaxNumGoal();             // update max number of goal
        void initPoseTable();               // initialize the pose table

        void updatePoseTable();             // update the pose table
        void startNavi();                   // start navigate for the first pose
        void cancelNavi();

        void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose);  //goal count sub callback function

        void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses); //status sub callback function

        void checkCycle();

        void completeNavi();               //after the first pose, continue to navigate the rest of poses
        void cycleNavi();

        bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list);  // check whether arrived the goal

        static void startSpin(); // spin for sub
        void InitTableItem(int num);
        void UpdateLine(QTableWidgetItem *item);
        void DeleteCubeMark(int id);
        void svaeNameFile();

        void retrieveMap();
    protected:
        QLineEdit *output_maxNumGoal_editor_;
        QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_cancel_button_;
        QTableWidget *poseArray_table_;
        QCheckBox *cycle_checkbox_;
        QDialog *dialog;

        QHBoxLayout *mainlayout = NULL;
          QTabWidget *tabWg_ = NULL;
            QWidget *tab0 = NULL;
              QHBoxLayout *tab0_horizontalLayout = NULL;//mapping
                QVBoxLayout *tab0_verticalLayout = NULL;
                  QComboBox *map_table = NULL;
                  QPushButton *record_map_btn = NULL;
                  QPushButton *start_mapping_btn = NULL;
                  QPushButton *stop_mapping_btn = NULL;
            QWidget *tab1 = NULL;
              QHBoxLayout *tab1_horizontalLayout = NULL;//navigation

        QLineEdit *file_line = NULL;
        QPushButton *save_file_btn = NULL;
        QString output_maxNumGoal_ = NULL;

        // The ROS node handle.
        ros::NodeHandle nh_;
        ros::Publisher goal_pub_, cancel_pub_, marker_pub_;
        ros::Subscriber goal_sub_, status_sub_;


        int maxNumGoal_;
        int curGoalIdx_ = 0, cycleCnt_ = 0;
        bool permit_ = false, cycle_ = false, arrived_ = false;
        int record_switch = 0;
        std::string file_name_str;
        geometry_msgs::PoseArray pose_array_;
        char *tmp;
        std::string work_dir;

        actionlib_msgs::GoalID cur_goalid_;
        QTableWidgetItem *table_item;
        int count = 0;
        bool write_flag = false;
        int map_flag = 0;
        int nav_switch = 0;
        int move_base_switch = 0;
        int amcl_switch = 0;
        int gmapping_switch = 0;
        int rf20_switch = 0;
        int lidar_switch = 0;
        int bunker_base_switch = 0;
    };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
