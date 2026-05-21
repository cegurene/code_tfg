#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "rl_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rl_interfaces__msg__MotionCommand() -> *const std::ffi::c_void;
}

#[link(name = "rl_interfaces__rosidl_generator_c")]
extern "C" {
    fn rl_interfaces__msg__MotionCommand__init(msg: *mut MotionCommand) -> bool;
    fn rl_interfaces__msg__MotionCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotionCommand>, size: usize) -> bool;
    fn rl_interfaces__msg__MotionCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotionCommand>);
    fn rl_interfaces__msg__MotionCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotionCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<MotionCommand>) -> bool;
}

// Corresponds to rl_interfaces__msg__MotionCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drive_ids: rosidl_runtime_rs::Sequence<u32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_position: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_velocity: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_torque: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for MotionCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rl_interfaces__msg__MotionCommand__init(&mut msg as *mut _) {
        panic!("Call to rl_interfaces__msg__MotionCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotionCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rl_interfaces__msg__MotionCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rl_interfaces__msg__MotionCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rl_interfaces__msg__MotionCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotionCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotionCommand where Self: Sized {
  const TYPE_NAME: &'static str = "rl_interfaces/msg/MotionCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rl_interfaces__msg__MotionCommand() }
  }
}


