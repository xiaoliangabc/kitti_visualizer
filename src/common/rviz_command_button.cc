#include "common/rviz_command_button.h"

namespace kitti_visualizer {
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
CommandButton::CommandButton(QWidget* parent) : rviz::Panel(parent) {
  // Get button parameters
  std::string current_path = ros::package::getPath("kitti_visualizer");
  std::string button_layout_file = current_path + "/param/button_layout.yaml";
  YAML::Node node = YAML::LoadFile(button_layout_file);
  std::vector<std::string> button_names;
  for (int i = 0; i < node.size(); ++i) {
    button_names.push_back(node[i]["name"].as<std::string>());
  }

  // Lay out buttons using QPushButton in a QHBoxLayout.
  QHBoxLayout* layout = new QHBoxLayout;
  QSignalMapper* signal_mapper = new QSignalMapper(this);
  for (auto button_name : button_names) {
    QPushButton* button = new QPushButton(QString::fromStdString(button_name));
    layout->addWidget(button);
    button->setEnabled(true);
    connect(button, SIGNAL(clicked()), signal_mapper, SLOT(map()));
    signal_mapper->setMapping(button, QString::fromStdString(button_name));
  }
  connect(signal_mapper, SIGNAL(mapped(QString)), this,
          SLOT(ButtonResponse(QString)));

  // Lay out the "frame string" text entry field using a QLabel and a QLineEdit
  // in a QHBoxLayout.
  QHBoxLayout* frame_layout = new QHBoxLayout;
  frame_layout->addWidget(new QLabel("Frame string:"));
  output_frame_editor_ = new QLineEdit;
  frame_layout->addWidget(output_frame_editor_);
  output_frame_editor_->setPlaceholderText("000000");
  output_frame_editor_->setText("000000");
  layout->addLayout(frame_layout);
  connect(output_frame_editor_, SIGNAL(returnPressed()), this,
          SLOT(UpdateFrame()));

  setLayout(layout);

  // Publisher
  command_publisher_ =
      nh_.advertise<std_msgs::String>("/kitti_visualizer/command_button", 1);
}

void CommandButton::ButtonResponse(QString command) {
  std_msgs::String command_msg;
  command_msg.data = command.toStdString();
  command_publisher_.publish(command_msg);
}

void CommandButton::UpdateFrame() {
  std_msgs::String command_msg;
  command_msg.data = output_frame_editor_->text().toStdString();
  command_publisher_.publish(command_msg);
  std::cout << output_frame_editor_->text().toStdString() << std::endl;
}

// Save all configuration data from this panel to the given Config object.
void CommandButton::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void CommandButton::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // kitti_visualizer

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kitti_visualizer::CommandButton, rviz::Panel)
