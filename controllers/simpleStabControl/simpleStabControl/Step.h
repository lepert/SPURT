#ifndef STEP_H
#define STEP_H
#include "Gait.h"
#include "Observer.h"
#include <boost/shared_ptr.hpp>
#include "Common.h"
#include "MMath.h"


enum Foot {
	LEFT_FOOT = 0,
	RIGHT_FOOT
};

enum StepType {
	REGULAR_STEP=0,
	//START_STEP,
	END_STEP
	//NULL_STEP
};

struct WalkVector {
	float x;
	float y;
	float theta;
};

static const WalkVector ZERO_WALKVECTOR = {0.0f,0.0f,0.0f};

class Step
{
public:
	Step(const WalkVector &target,
		const Gait & gait,	 
		const Foot _foot,
		const WalkVector &last = ZERO_WALKVECTOR,
		const StepType _type = REGULAR_STEP);

		 Step(const Step &);


		
		 Step(const float new_x, const float new_y, const float new_theta,
			 const Step& other);


	~Step(void);

	float x;
	float y;
	float theta;
	WalkVector walkVector;
	unsigned int stepDurationFrames;
	unsigned int doubleSupportFrames;
	unsigned int singleSupportFrames;
	float sOffsetY;
	Foot foot;
	StepType type;
	bool zmpd;

	const WalkVector accelClipVelocities(const WalkVector & source,
		const WalkVector & last);
	    const WalkVector lateralClipVelocities(const WalkVector & source);
	float stepConfig[GaitDefaults::LEN_STEP_CONFIG];
	float zmpConfig[GaitDefaults::LEN_ZMP_CONFIG];
	float stanceConfig[GaitDefaults::LEN_STANCE_CONFIG];

	void updateFrameLengths(const float duration,
		const float dblSuppF);
	void setStepSize(const WalkVector &target,
		const WalkVector &last);

private:

	  void copyAttributesFromOther(const Step &other);
	void copyGaitAttributes(const float _step_config[],
		const float _zmp_config[],
		const float _stance_config[]);
	   const WalkVector elipseClipVelocities(const WalkVector & source);
	   void setStepLiftMagnitude();
};

static const std::tr1::shared_ptr<Step> EMPTY_STEP(new Step(ZERO_WALKVECTOR,
	DEFAULT_GAIT,
	LEFT_FOOT));

#endif