import math

UPPER_ARM_LENGTH = 462.792  # mm
LOWER_ARM_LENGTH = 430.994  # mm
# UPPER_ARM_LENGTH = 564.243  # mm
# LOWER_ARM_LENGTH = 519.479  # mm


# CONVERT ENCODER VALUES TO POSE OF THE ARM END EFFECTOR

def encoder_to_angle_upper(encoder_value):
    angle_degrees = -0.02863 * encoder_value + 101.58735
    return angle_degrees


def encoder_to_angle_lower(encoder_value):
    angle_degrees = 0.05689 * encoder_value - 87.22924
    return angle_degrees


def encoder_to_pose(upper_encoder, lower_encoder):
    upper_angle = encoder_to_angle_upper(upper_encoder)
    lower_angle = encoder_to_angle_lower(lower_encoder)

    total_x = LOWER_ARM_LENGTH * math.sin(math.radians(lower_angle)) + UPPER_ARM_LENGTH * math.sin(math.radians(lower_angle + upper_angle))
    total_z = LOWER_ARM_LENGTH * math.cos(math.radians(lower_angle)) + UPPER_ARM_LENGTH * math.cos(math.radians(lower_angle + upper_angle))
    
    return total_x, total_z


# CONVERT POSE OF THE ARM END EFFECTOR TO ENCODER VALUES

def angle_upper_to_encoder(angle_degrees):
    encoder_value = -34.84286 * angle_degrees + 3541.92857
    return encoder_value


def angle_lower_to_encoder(angle_degrees):
    encoder_value = 17.51429 * angle_degrees + 1534.28571
    return encoder_value


def arm_angle_from_pose(x, z):
    pose_len = math.sqrt(x**2 + z**2)

    varA1 = (pose_len**2 + LOWER_ARM_LENGTH**2 - UPPER_ARM_LENGTH**2) / (
        2 * pose_len * LOWER_ARM_LENGTH)

    var_angA1 = math.degrees(math.acos(varA1))

    theta_base = math.degrees(math.atan2(x, z))
    lower_angle_degrees = theta_base - var_angA1

    varA2 = (UPPER_ARM_LENGTH**2 + LOWER_ARM_LENGTH**2 - pose_len**2) / (
        2 * UPPER_ARM_LENGTH * LOWER_ARM_LENGTH)

    upper_angle_degrees = 180 - math.degrees(math.acos(varA2))

    return upper_angle_degrees, lower_angle_degrees


def pose_to_encoder(x, z):
    upper_angle, lower_angle = arm_angle_from_pose(x, z)
    upper_encoder = int(angle_upper_to_encoder(upper_angle))
    lower_encoder = int(angle_lower_to_encoder(lower_angle))
    return upper_encoder, lower_encoder


# Example usage
pose_x = UPPER_ARM_LENGTH   # mm
pose_z = LOWER_ARM_LENGTH  # mm

upper_enc, lower_enc = pose_to_encoder(pose_x, pose_z)
print(f"Upper Encoder: {upper_enc}, Lower Encoder: {lower_enc}")
x, z = encoder_to_pose(upper_enc, lower_enc)
print(f"Pose X: {x}, Pose Z: {z}")
