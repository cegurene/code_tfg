#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to rl_interfaces__msg__MotionCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drive_ids: Vec<u32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_position: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_velocity: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_torque: Vec<f32>,

}



impl Default for MotionCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotionCommand::default())
  }
}

impl rosidl_runtime_rs::Message for MotionCommand {
  type RmwMsg = super::msg::rmw::MotionCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drive_ids: msg.drive_ids.into(),
        target_position: msg.target_position.into(),
        target_velocity: msg.target_velocity.into(),
        target_torque: msg.target_torque.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drive_ids: msg.drive_ids.as_slice().into(),
        target_position: msg.target_position.as_slice().into(),
        target_velocity: msg.target_velocity.as_slice().into(),
        target_torque: msg.target_torque.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      drive_ids: msg.drive_ids
          .into_iter()
          .collect(),
      target_position: msg.target_position
          .into_iter()
          .collect(),
      target_velocity: msg.target_velocity
          .into_iter()
          .collect(),
      target_torque: msg.target_torque
          .into_iter()
          .collect(),
    }
  }
}


