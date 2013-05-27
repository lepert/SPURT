#ifndef GAIT_DEFAULTS_H
#define GAIT_DEFAULTS_H

namespace GaitDefaults{


	

	/**
 * STANCE CONFIG параметры:
 *  bodyHeight      -- высота CoM  
 *  bodyOffsetY     -- смещеие вперед CoM
 *  legSeparationY  -- расстояние мд ногами
 *  bodyRotationY   -- угол по У тела
 *  legRotationY    -- угол между ногами по z
 *
 */
    enum StanceConfig {
        BODY_HEIGHT=0,
        BODY_OFF_X,
        LEG_SEPARATION_Y,
        BODY_ROT_Y,
        LEG_ROT_Z,
        TRANS_TIME,
        LEN_STANCE_CONFIG
    };

/**
 * STEP CONFIG
 *  stepDuration         -- время на шаг
 *  dblSupportPercent   -- доля времени в стадии полной поддержки
 *  stepHeight           -- высота шага в процессе хотьбы
 *  maxVelX              -- 
 *  maxVelY              -- 
 *  maxVelTheta          -- максимальная угловая скорость
 */
    enum StepConfig{
        DURATION,
        DBL_SUPP_P,
        STEP_HEIGHT, 
	FOOT_LIFT_ANGLE,
        MAX_VEL_X,
        MIN_VEL_X,
        MAX_VEL_Y,
        MAX_VEL_THETA,
        MAX_ACC_X,
        MAX_ACC_Y,
        MAX_ACC_THETA,
	WALKING, //1.0  шагать -другое не шагать
        LEN_STEP_CONFIG
    };

    static const float NON_WALKING_GAIT = 0.0f;
    static const float WALKING_GAIT = 1.0f;

/**
 * ZMP CONFIG parameters:
 *  footCenterX                   -- 
 *  doubleSupportStaticPercentage -- 
 *  lZMPOffY                -- 
 *  rZMPOffY               -- 
 *  strafeZMPOff               -- 
 *  turnZMPOff                 -- 
 *
 */
    enum ZmpConfig{
        FOOT_CENTER_X=0,
        DBL_SUP_STATIC_P,
        L_ZMP_OFF_Y,
        R_ZMP_OFF_Y,
        STRAFE_ZMP_OFF,
        TURN_ZMP_OFF,
        LEN_ZMP_CONFIG
    };

/**
 * JOINT HACK CONFIG
 *  lHipAmplitude  -- магнитуда угла, который добавляется в теч шага
 *  rHipAmplitude -- 
 */
    enum JointHackConfig{
        L_HIP_AMP=0,
        R_HIP_AMP,
        LEN_HACK_CONFIG
    };

/**
 * SENSOR CONFIG
 * observerScale   -- 
 * angleScale      -- 
 */
    enum SensorConfig{
        FEEDBACK_TYPE=0,
        GAMMA_X,
        GAMMA_Y,
        SPRING_K_X,
        SPRING_K_Y,
        MAX_ANGLE_X,
        MAX_ANGLE_Y,
        MAX_ANGLE_VEL,
        LEN_SENSOR_CONFIG
    };

/**
 * STIFFNESS CONFIG
 * hipStiff     --  hip
 * KPStiff      --  knee pitch
 * APStiff      --  ankle pitch
 * ARStiff      -- ankle roll
 * armStiff     --  arms
 */
    enum StiffnessConfig{
        HIP = 0,
        KP,
        AP,
        AR,
		ARM,
        ARM_PITCH,
        LEN_STIFF_CONFIG
    };


    enum OdoConfig{
        X_SCALE = 0,
        Y_SCALE,
        THETA_SCALE,
        LEN_ODO_CONFIG
    };

/**
 * ARM CONFIG
 * armAmplitude -- амплитуда угла
 */
    enum ArmConfig{
        AMPLITUDE = 0,
        LEN_ARM_CONFIG
    };
	static const float toCM=10;
	static const float toRad=3.14/180;

	static const float STANCE_DEFAULT[LEN_STANCE_CONFIG]=
	{31.00*toCM, // CoM height
	1.45*toCM,  // Forward displacement of CoM
	10.0*toCM,  // Horizontal distance between feet
	3.0*toRad,   // Body angle around y axis
	0.0,   // Angle between feet
	0.1};//время перехода
	static const float STEP_DEFAULT[LEN_STEP_CONFIG]=
	{0.3, // step duration
	0.1,  // fraction in double support
	1.5*toCM,  // stepHeight
	-5.0*toRad,  // step lift
	40.0*toCM,  // max x speed
	-40.0*toCM,  // max x speed
	40.0*toCM,  // max y speed
	45.0*toRad, // max theta speed()
	40.0*toCM,  // max x accel
	40.0*toCM,  // max y accel
	20.0*toRad,
	WALKING_GAIT};
	static const float ZMP_DEFAULT[LEN_ZMP_CONFIG]=
	{0.0,  // footCenterLocX
	0.3,  // zmp static percentage
	0.45*toCM,  // left zmp off
	0.45*toCM,  // right zmp off
	0.001,  // strafe zmp offse
	6.6};//
	static const float HACK_DEFAULT[LEN_HACK_CONFIG]=
	{0.1f,//l
	0.1f};// r
	static const float SENSOR_DEFAULT[LEN_SENSOR_CONFIG]=
	{0.0,   // Feedback type (1.0 = spring, 0.0 = old)
	0.06,  // angle X scale (gamma)
	0.08,  // angle Y scale (gamma)
	250.0,  // X spring constant k (kg/s^2)
	100.0,  // Y spring constant k (kg/s^2)
	7.0,   // max angle X (compensation)
	7.0,   // max angle Y
	45.0};
	static const float STIFF_DEFAULT[LEN_STIFF_CONFIG]=
	{0.85f,//hip
	0.3f,//knee
	0.4f,//ap
	0.3f,//ar
	0.2f,//arm
	0.2f};//arm pitch
	static const float ODO_DEFAULT[LEN_ODO_CONFIG]=
	{1.0f,//xodo
	1.0f,//yodo
	1.0f};//thetaodo
	static const float ARM_DEFAULT[LEN_ARM_CONFIG]=
	{0.5f};//амплитуда руки

};

#endif//GAIT_DEFAULTS_H