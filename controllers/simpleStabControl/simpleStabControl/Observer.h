#ifndef OBSERVER_H
#define OBSERVER_H
#include <Eigen/Core>
#include <list>

class Observer 
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Observer(void);
	~Observer(void);
	 const float tick(const std::list<float> *zmp_ref,
		const float cur_zmp_ref,
		const float sensor_zmp);
	void initState(float x, float v, float p);
	static const unsigned int NUM_PREVIEW_FRAMES = 70;
	    static const unsigned int NUM_AVAIL_PREVIEW_FRAMES = 120;
		 const float getPosition() const { return stateVector(0); }
		 
private:
	
	Eigen::Vector3f stateVector;
	float trackingError;
	Eigen::Vector3f mStateVector;
	float mTrackingError;
	static const float weights[NUM_AVAIL_PREVIEW_FRAMES];
	Eigen::Matrix3f A;
	Eigen::Vector3f b;
	Eigen::Vector3f c;
	Eigen::Vector3f L;
	

		static const float A_values[9];
		static const float b_values[3];
		static const float c_values[3];
		static const float L_values[3];
		static const float Gi;
};

#endif //OBSERVER_H