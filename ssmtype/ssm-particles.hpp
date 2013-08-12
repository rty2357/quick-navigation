/*
 * @file particles.h
 */


#ifndef __PARTICLES_HPP__
#define __PARTICLES_HPP__

#include <stdint.h>

#include <ssmtype/spur-odometry.h>

#include "gnd-random.hpp"

#include "gnd-matrix-base.hpp"
#include "gnd-queue.hpp"

#define SNAME_PARTICLES					"ssm_particles"
#define SNAME_PARTICLES_EVALUATION		"ssm_particle_eval"


enum {
	PARTICLE_X				= 0,
	PARTICLE_Y				= 1,
	PARTICLE_THETA			= 2,
	PARTICLE_WHEEL_MEAN		= 3,
	PARTICLE_WHEEL_RATIO	= 4,	// ratio ( left / right )
	PARTICLE_GYRO_BIAS 		= PARTICLE_WHEEL_RATIO,
	PARTICLE_TREAD_RATIO	= 5,	// ratio ( tread / wheel mean)
	PARTICLE_GYRO_SF 		= PARTICLE_TREAD_RATIO,
	PARTICLE_EVAL			= 6,
	PARTICLE_END			= 7
};

enum {
	PARTICLE_POS_INDX = PARTICLE_X,
	PARTICLE_PROP_INDX = PARTICLE_WHEEL_MEAN,
	PARTICLE_EVAL_INDX = PARTICLE_EVAL
};

enum {
	PARTICLE_POS_DIM = 3,
	PARTICLE_PROP_DIM = 3,
	PARTICLE_EVAL_DIM = 1,
	PARTICLE_DIM = PARTICLE_END
};


struct POSITION_PARTICLE {
	union {
		struct {
			struct {
				double x;
				double y;
				double theta;
			} odo;
			union {
				struct {
					double wheel_mean;
					double wheel_ratio;
					double tread_ratio;
				} wheel_odm;
				struct {
					double wheel_mean;
					double bias;
					double sf;
				} gyro_odm;
			} prop;
			double eval;
		};
		double vector[PARTICLE_DIM];
	};
};
typedef struct POSITION_PARTICLE position_particle_t;





/**
 * @brief particle set
 */
class POSITION_PARTICLE_SET_CLASS
	: public gnd::queue< gnd::matrix::fixed< 1, PARTICLE_DIM > >
{
// ---> const definition
public:
	typedef double		value_t;

// ---> consturctor,destructor
public:
	POSITION_PARTICLE_SET_CLASS();
	~POSITION_PARTICLE_SET_CLASS();

// ---> robot position
public:
	position_particle_t pos;

// ---> init
public:
	template < typename MTRX1, typename MTRX2, typename MTRX3 >
	int init_particle( MTRX1 *myu, MTRX2 *sigmap, MTRX3 *sigmak, const uint32_t n );

// ---> kinematics
public:
	int init_kinematics( double cnt_rv, double gr );
protected:
	/**
	 * @brief kinematics
	 */
	struct {
		double enc_rev;
	} _mtr;

// ---> motion
public:
	int odometry_motion(int re, int le);
	int gyro_odometry_motion(int re, int le, double vol, double dt);

// ---> resampling
public:
	/**
	 * @brief resampling remain
	 */
	int resampling_remain(double *eval, const uint32_t n, value_t* ave = 0);

	/**
	 * @brief resampling change position
	 */
	template < typename MTRX >
	int resampling_position( MTRX *sigma, const uint32_t n);
	/**
	 * @brief resampling change kinematics
	 */
	template < typename MTRX >
	int resampling_kinematics( MTRX *sigma, const uint32_t n);
	/**
	 * @brief resampling change kinematics
	 */
	template < typename MTRX1, typename MTRX2 >
	int resampling_kinematics_reset( MTRX1 *myu, MTRX2 *sigma, const uint32_t n);
	/**
	 * @brief
	 */
	template< typename MTRX >
	int random_sampling( MTRX *myu );


private:
	/**
	 * @brief strage for resampling
	 */
	struct {
		gnd::queue< gnd::matrix::fixed< 1, PARTICLE_DIM > > storage;
		double sum;
	} _resampling_var;
};
typedef POSITION_PARTICLE_SET_CLASS particle_set_c;


/*
 * @brief constructor
 */
inline POSITION_PARTICLE_SET_CLASS::POSITION_PARTICLE_SET_CLASS()
{
	_mtr.enc_rev = 0;
}



/*
 * @brief destructor
 */
inline POSITION_PARTICLE_SET_CLASS::~POSITION_PARTICLE_SET_CLASS()
{
}



/**
 * @brief generate initial particle set
 * @param [in] myu		: mean
 * @param [in] sigmap	: position covariance matrix
 * @param [in] sigmak	: kinematics covariance matrix
 * @param [in] n		: number of particle
 */
template < typename MTRX1, typename MTRX2, typename MTRX3 >
inline int POSITION_PARTICLE_SET_CLASS::init_particle( MTRX1 *myu, MTRX2 *sigmap, MTRX3 *sigmak, const uint32_t n )
{
	gnd_assert(!myu || !sigmap, -1, "null pointer");
	gnd_assert(gnd::vector::size(myu) < (signed)PARTICLE_DIM, -1, "invalid argument");
	gnd_assert(gnd::matrix::row(sigmap) < (signed) PARTICLE_POS_DIM, -1, "invalid argument");
	gnd_assert(gnd::matrix::column(sigmap) < (signed) PARTICLE_POS_DIM, -1, "invalid argument");
	gnd_error(n <= 0, -1, "no effect argument");


	{ // ---> initialize matrix
		clear();
		__reallocate__(n);
	} // <--- initialize matrix

	{
		uint32_t i = 0;
		gnd::matrix::fixed<1, PARTICLE_DIM> rnd;
		gnd::matrix::fixed<1, PARTICLE_POS_DIM> ws;
		gnd::matrix::fixed<PARTICLE_POS_DIM, PARTICLE_POS_DIM> cp_sigma;

		// set
		push_back( myu );
		::memcpy(&pos, gnd::matrix::pointer(myu, 0, 0), sizeof(pos));

		pos.odo.x = pos.odo.y = pos.odo.theta = 0;
		pos.prop.wheel_odm.wheel_mean = pos.prop.wheel_odm.wheel_ratio = pos.prop.wheel_odm.tread_ratio = 0;

		for(i = 1; i < n; i++){
			gnd::matrix::flex rand_ref;			// reference for randamize
			gnd::matrix::set_zero(&rnd);

			{ // ---> add random (position)
				gnd::matrix::assign_to_as_vector(&rand_ref, &rnd, 0, PARTICLE_POS_INDX, PARTICLE_POS_DIM);
				gnd::matrix::copy(&cp_sigma, sigmap);
				gnd::random_gaussian_mult(&cp_sigma, PARTICLE_POS_DIM, &ws, &rand_ref);
			} // <--- add random (position)
			// ---> add random (kinematics)
			if( sigmak ) {
				gnd::matrix::assign_to_as_vector(&rand_ref, &rnd, 0, PARTICLE_PROP_INDX, PARTICLE_PROP_DIM);
				gnd::matrix::copy(&cp_sigma, sigmak);
				gnd::random_gaussian_mult(&cp_sigma, PARTICLE_PROP_DIM, &ws, &rand_ref);
			} // <--- add random (kinematics)
			// add random error to mean
			gnd::matrix::add(&rnd, myu, &rnd);
			// set new particle
			push_back( &rnd );

			pos.odo.x += rnd[0][PARTICLE_X];
			pos.odo.y += rnd[0][PARTICLE_Y];
			pos.odo.theta += rnd[0][PARTICLE_THETA];
			pos.prop.wheel_odm.wheel_mean += rnd[0][PARTICLE_WHEEL_MEAN];
			pos.prop.wheel_odm.wheel_ratio += rnd[0][PARTICLE_WHEEL_RATIO];
			pos.prop.wheel_odm.tread_ratio += rnd[0][PARTICLE_TREAD_RATIO];
		}

		pos.odo.x /= n;
		pos.odo.y /= n;
		pos.odo.theta /= n;
		pos.prop.wheel_odm.wheel_mean /= n;
		pos.prop.wheel_odm.wheel_ratio /= n;
		pos.prop.wheel_odm.tread_ratio /= n;
	}

//	_n = n;
	return 0;
}



/**
 * @brief kinematics
 * @param [in] cnt_rsl 	: resolution of encoder
 * @param [in] gr		: gear ratio
 */
inline int POSITION_PARTICLE_SET_CLASS::init_kinematics( double cnt_rsl, double gr )
{
	_mtr.enc_rev = cnt_rsl * gr;
	return 0;
}



/**
 * @brief odometry motion
 * @param [in] cnt1	: encoder count (left)
 * @param [in] cnt2	: encoder count (right)
 */
inline int POSITION_PARTICLE_SET_CLASS::odometry_motion(int cnt1, int cnt2)
{
	gnd_error(_mtr.enc_rev == 0, -1, "invalid property (encoder resolution)");

	{ // ---> operation
		double wr, wl;
		uint32_t i;

		{ // ---> compute wheel rotation quantity
			wr = ( 2.0 * M_PI * ( (double) cnt2 ) ) / (_mtr.enc_rev );
			wl = ( 2.0 * M_PI * ( (double) cnt1 ) ) / (_mtr.enc_rev );
		} // <--- compute wheel rotation quantit

		{
			double wr2 = wr * ( 2.0 - pos.prop.wheel_odm.wheel_ratio );
			double wl2 = wl * pos.prop.wheel_odm.wheel_ratio;
			// robot translation quantity
			double tq = ( pos.prop.wheel_odm.wheel_mean * ( wr2 + wl2 ) ) / 2.0;
			// robot rotation quantity
			double rq = ( wr2 - wl2 ) / pos.prop.wheel_odm.tread_ratio ;

			pos.odo.x += tq * ::cos(pos.odo.theta);
			pos.odo.y += tq * ::sin(pos.odo.theta);
			pos.odo.theta += rq;
		}

		for(i = 0; (signed)i < size(); i++){
			double wr2 = wr * ( 2.0 - (*this)[i][0][PARTICLE_WHEEL_RATIO] );
			double wl2 = wl * (*this)[i][0][PARTICLE_WHEEL_RATIO];
			// robot translation quantity
			double tq = ( (*this)[i][0][PARTICLE_WHEEL_MEAN] * ( wr2 + wl2 ) ) / 2.0;
			// robot rotation quantity
			double rq = ( wr2 - wl2 ) / (*this)[i][0][PARTICLE_TREAD_RATIO];

			(*this)[i][0][PARTICLE_X] += tq * ::cos((*this)[i][0][PARTICLE_THETA]);
			(*this)[i][0][PARTICLE_Y] += tq * ::sin((*this)[i][0][PARTICLE_THETA]);
			(*this)[i][0][PARTICLE_THETA] += rq;
		}

	} // <--- operation

	return 0;
}


/**
 * @brief odometry motion
 * @param [in] cnt1	: encoder count (left)
 * @param [in] cnt2	: encoder count (right)
 * @param [in] gyro : gyro sensor reading (gyro)
 */
inline int POSITION_PARTICLE_SET_CLASS::gyro_odometry_motion(int cnt1, int cnt2, double gyro, double dt)
{
	gnd_error(_mtr.enc_rev == 0, -1, "invalid property (encoder resolution)");

	{ // ---> operation
		double qt, qr;
		uint32_t i;

		qt = (pos.prop.gyro_odm.wheel_mean * M_PI * ( (double) cnt1 + cnt2 ) / (_mtr.enc_rev) );
		if(cnt1 == 0 && cnt2 == 0) {
			qr = 0;
			pos.prop.gyro_odm.bias += (gyro - pos.prop.gyro_odm.bias) * 0.001;
		}
		else {
			qr = -(gyro - pos.prop.gyro_odm.bias) * pos.prop.gyro_odm.sf * dt;
		}

		{
			pos.odo.x += qt * ::cos(pos.odo.theta);
			pos.odo.y += qt * ::sin(pos.odo.theta);
			pos.odo.theta += qr;
		}

		for(i = 0; (signed)i < size(); i++){
			qt = ( (*this)[i][0][PARTICLE_WHEEL_MEAN] * M_PI * ( (double) cnt1 + cnt2 ) / (_mtr.enc_rev) );
			if(cnt1 == 0 && cnt2 == 0) {
				qr = 0;
				(*this)[i][0][PARTICLE_GYRO_BIAS] += (gyro - (*this)[i][0][PARTICLE_GYRO_BIAS] * 0.001 );
			}
			else {
				qr = -(gyro - (*this)[i][0][PARTICLE_GYRO_BIAS] ) * (*this)[i][0][PARTICLE_GYRO_SF] * dt;
			}

			(*this)[i][0][PARTICLE_X] += qt * ::cos( (*this)[i][0][PARTICLE_THETA] );
			(*this)[i][0][PARTICLE_Y] += qt * ::sin( (*this)[i][0][PARTICLE_THETA] );
			(*this)[i][0][PARTICLE_THETA] += qr;
		}

	} // <--- operation

	return 0;
}





/**
 * @brief init_resampling()
 */
inline int POSITION_PARTICLE_SET_CLASS::resampling_remain(double *eval, const uint32_t nremain, value_t *ave)
{
	double evalsum = 0;
	{ // ---> initialize
		_resampling_var.storage.clear();
		_resampling_var.sum = 0;
	} // <--- initialize

	{ // ---> set evaluation and compute sum
		uint32_t i;

		// compute eval
		for(i = 0; (signed)i < size(); i++){
			evalsum += eval[i];
		}

		if(evalsum <= 0)	return -1;
	} // <--- set evaluation and compute sum

	{ // ---> resampling remain
		uint32_t i, j;
		double rnd;
		gnd::matrix::fixed<1, PARTICLE_DIM> wave;
		gnd::matrix::fixed<1, PARTICLE_DIM> ws;
		gnd::matrix::flex ref;

		// ---> save into storage
		gnd::matrix::set_zero(&wave);
		for(i = 0; i < nremain; i++){
			double sum = 0;
			rnd = evalsum * gnd::random_uniform();
			// select particle
			for(j = 0; (signed)j < size() && rnd > sum; j++)	sum += eval[j];
			if(j >= 0)	j--;

			_resampling_var.storage.push_back( (*this) + j );
			// increment remaining count
			_resampling_var.storage[i][0][PARTICLE_EVAL] = eval[j];

			// summation of evaluation
			_resampling_var.sum += eval[j];

			// weighted summation of particle vector
			gnd::matrix::clear(&ref);

			gnd::matrix::scalar_prod(_resampling_var.storage + i,  eval[j],  &ws);
			gnd::matrix::add(&wave, &ws, &wave);
		} // for(i)
		// <--- save into storage

		// weighted average
		gnd::matrix::scalar_div(&wave, _resampling_var.sum, &wave);
		::memcpy(&pos, gnd::matrix::pointer(&wave, 0, 0), sizeof(pos));

		// ---> set remaining particle
		clear();
		for(i = 0; (signed)i < _resampling_var.storage.size(); i++){
			push_back( _resampling_var.storage + i );
		}
		// <--- set remaining particle

		// save eval average
		if(ave){
			*ave = _resampling_var.sum / _resampling_var.storage.size();
		}

	} // <--- resampling remain

	return 0;
}



/**
 * @brief resampling change position
 */
template < typename MTRX >
inline int POSITION_PARTICLE_SET_CLASS::resampling_position( MTRX *sigma, const uint32_t n)
{
	gnd::matrix::fixed<1, PARTICLE_DIM> tmp;
	gnd::matrix::fixed<1, PARTICLE_POS_DIM> ws;
	gnd::matrix::fixed<PARTICLE_POS_DIM, PARTICLE_POS_DIM> cp_sigma;
	uint32_t i, j;
	double rnd;

	for(i = 0; i < n; i++){
		double sum = 0;
		rnd = _resampling_var.sum * gnd::random_uniform();

		// select perticle
		for(j = 0; (signed)j < _resampling_var.storage.size() && rnd > sum; j++)
			sum += _resampling_var.storage[j][0][PARTICLE_EVAL];
		j--;

		{ // ---> add random (position)
			gnd::matrix::flex rndp;

			// clear
			gnd::matrix::set_zero(&tmp);
			// assign
			gnd::matrix::assign_to_as_vector(&rndp, &tmp, 0, PARTICLE_POS_INDX, PARTICLE_POS_DIM);
			gnd::matrix::copy(&cp_sigma, sigma);
			// generate random
			gnd::random_gaussian_mult(&cp_sigma, PARTICLE_POS_DIM, &ws, &rndp);

			gnd::matrix::add(&tmp, _resampling_var.storage + j, &tmp);
			// initialize remaining count
			push_back( &tmp );
		} // <--- add random (position)
	} // for(i)

	return 0;
}


/**
 * @brief resampling change kinematics
 */
template < typename MTRX >
inline int POSITION_PARTICLE_SET_CLASS::resampling_kinematics( MTRX *sigma, const uint32_t n)
{
	gnd::matrix::fixed<1, PARTICLE_DIM> tmp;
	gnd::matrix::fixed<1, PARTICLE_PROP_DIM> ws;
	gnd::matrix::fixed<PARTICLE_PROP_DIM, PARTICLE_PROP_DIM> cp_sigma;
	uint32_t i, j;
	double rnd;

	for(i = 0; i < n; i++){
		double sum = 0;
		rnd = _resampling_var.sum * gnd::random_uniform();

		// select perticle
		for(j = 0; (signed)j < _resampling_var.storage.size() && rnd > sum; j++)
			sum += _resampling_var.storage[j][0][PARTICLE_EVAL];
		j--;

		{ // ---> add random (position)
			gnd::matrix::flex rndp;

			// clear
			gnd::matrix::set_zero(&tmp);
			// assign
			gnd::matrix::assign_to_as_vector(&rndp, &tmp, 0, PARTICLE_PROP_INDX, PARTICLE_PROP_DIM);
			gnd::matrix::copy(&cp_sigma, sigma);
			// generate random
			gnd::random_gaussian_mult(&cp_sigma, PARTICLE_PROP_DIM, &ws, &rndp);

			gnd::matrix::add(&tmp, _resampling_var.storage + j, &tmp);
			// initialize remaining count
			push_back( &tmp );
		} // <--- add random (position)
	} // for(i)

	return 0;
}

template < typename MTRX1, typename MTRX2 >
inline int POSITION_PARTICLE_SET_CLASS::resampling_kinematics_reset( MTRX1 *myu, MTRX2 *sigma, const uint32_t n)
{
	gnd::matrix::fixed<1, PARTICLE_DIM> tmp;
	gnd::matrix::fixed<1, PARTICLE_PROP_DIM> ws;
	gnd::matrix::fixed<PARTICLE_PROP_DIM, PARTICLE_PROP_DIM> cp_sigma;
	uint32_t i, j;
	double rnd;

	for(i = 0; i < n; i++){
		double sum = 0;
		rnd = _resampling_var.sum * gnd::random_uniform();

		// select perticle
		for(j = 0; (signed)j < _resampling_var.storage.size() && rnd > sum; j++)
			sum += _resampling_var.storage[j][0][PARTICLE_EVAL];
		j--;

		{ // ---> add random (position)
			gnd::matrix::flex rndp;

			// clear
			gnd::matrix::set_zero(&tmp);
			// assign
			gnd::matrix::assign_to_as_vector(&rndp, &tmp, 0, PARTICLE_PROP_INDX, PARTICLE_PROP_DIM);
			gnd::matrix::copy(&cp_sigma, sigma);
			// generate random
			gnd::random_gaussian_mult(&cp_sigma, PARTICLE_PROP_DIM, &ws, &rndp);

			//todo
			gnd::matrix::set(_resampling_var.storage + j, 0, PARTICLE_PROP_INDX, gnd::matrix::pointer(myu, 0, PARTICLE_PROP_INDX), PARTICLE_PROP_DIM);
			gnd::matrix::add(&tmp, _resampling_var.storage + j, &tmp);
			// initialize remaining count
			push_back( &tmp );
		} // <--- add random (position)
	} // for(i)

	return 0;
}






/**
 * @brief
 */
template< typename MTRX >
int POSITION_PARTICLE_SET_CLASS::random_sampling( MTRX *myu )
{
	if( nalloc() == size() )	return -1;

	push_back(gnd::matrix::pointer(myu, 0, 0), PARTICLE_DIM);

	return 0;
}



#include <ssm.hpp>

/**
 * @brief property of particles
 */
struct SSMParticlesProperty {
	int n;	//	number of allocate
	SSMParticlesProperty();
};
inline SSMParticlesProperty::SSMParticlesProperty()
{
	n = 0;
}


/**
 * @brief ssm-data particles
 */
class SSMParticles
	: public SSMApi<particle_set_c, SSMParticlesProperty>
{
public:
	SSMParticles(){}
	SSMParticles( const char *streamName, int streamId = 0 ) : SSMApi<particle_set_c, SSMParticlesProperty>( streamName, streamId ){}

	typedef gnd::matrix::fixed<1, PARTICLE_DIM> ssmdata_t;

public:
	///
	size_t sharedSize(  )
	{
		return sizeof(gnd::matrix::fixed<1, PARTICLE_DIM>) * data.nalloc();
	}


private:
	static void _ssmWrite( void *assmp, const void *adata, void *userData ){
//		ssmdata_t *ssmp = static_cast<ssmdata_t *>( assmp );
		const particle_set_c *data = static_cast<const particle_set_c*>(adata);
		uint64_t n = data->size();

		::memcpy( assmp, &n, sizeof(uint64_t));
		::memcpy( (char*)assmp + sizeof(uint64_t), data->const_begin(),
				data->size() * sizeof(ssmdata_t));
	}

	static void _ssmRead( const void *assmp, void *adata, void *auserData )
	{
		const ssmdata_t *ssmp = static_cast<const ssmdata_t *>( (const void*) ((const char*)assmp + sizeof(uint64_t)));
		particle_set_c *data = static_cast<particle_set_c *>( adata );
		uint64_t n;

		::memcpy( &n, static_cast<const char *>(assmp), sizeof(uint64_t) );
		if( data->nalloc() < n )	data->allocate(n);
		data->clear();
		data->push_back( ssmp,  n );
	}

public:
	bool write( ssmTimeT time = gettimeSSM() )
	{
		if( !isOpen(  ) )
			return false;
		int tid;
		tid = writeSSMP( ssmId, &data, time, SSMParticles::_ssmWrite, NULL );
		if(tid >= 0){timeId = tid; this->time = time; return true;}
		return false;
	}

	/// @brief tidを指定して読み込み
	/// @return 正しく読み込めたときtrueを返す
	bool read(int timeId = -1)
	{
		if( !isOpen(  ) )
			return false;
		int tid;
		tid = readSSMP( ssmId, &data, &time, timeId, SSMParticles::_ssmRead, NULL );
		if(tid >= 0){this->timeId = tid; return true;}
		return false;
	}


	/**
	 * @brief 時間指定読み込み
	 * @return 正しく読み込めたときtrueを返す
	 *
	 * @see readSSM_time, read
	 */
	bool readTime( ssmTimeT time )
	{
		if( !isOpen(  ) )
		{
			return false;
		}
		SSM_tid tid = readSSMP_time( ssmId, &data, time, &( this->time ),
				SSMParticles::_ssmRead, NULL );
		if( tid >= 0 )
		{
			timeId = tid;
			return true;
		}
		return false;
	}

};


class PARTICLE_EVALUATION {
public:
	PARTICLE_EVALUATION(){
		value = 0;
		n = 0;
	}
	~PARTICLE_EVALUATION(){
		deallocate();
	}

public:
	double *value;
	uint32_t n;

	int allocate( uint32_t i){
		value = new double[i];
		return 0;
	}

	int deallocate(){
		if(value)	delete[] value;
		else 	return -1;
		return 0;
	}

	int is_alloc(){
		return value != 0;
	}

};
typedef PARTICLE_EVALUATION particle_evaluation;




/**
 * @brief ssm-data particles
 */
class SSMParticleEvaluation
	: public SSMApi<particle_evaluation, SSMParticlesProperty>
{
public:
	SSMParticleEvaluation(){}
	SSMParticleEvaluation( const char *streamName, int streamId = 0 ) :
		SSMApi<particle_evaluation, SSMParticlesProperty>( streamName, streamId ){}
	typedef particle_evaluation ssmdata_t;

public:
	///
	size_t sharedSize(  )
	{
		return (sizeof( data.n ) + sizeof( double ) * data.n);
	}


private:
	static void _ssmWrite( void *assmp, const void *adata, void *userData ){
		const ssmdata_t *data = static_cast<const ssmdata_t *>( adata );
		char *p = (char*)assmp;
		int offset = 0;

		::memcpy( p + offset, &data->n, sizeof(data->n) );
		offset += sizeof(data->n);
		::memcpy( p + offset, data->value, sizeof(double) * data->n );
	}

	static void _ssmRead( const void *assmp, void *adata, void *auserData )
	{
		ssmdata_t *data = static_cast<ssmdata_t *>( adata );
		char *p = (char*)assmp;
		int offset = 0;

		::memcpy( &data->n, p + offset, sizeof(data->n) );
		offset += sizeof(data->n);
		::memcpy( data->value, p + offset, sizeof(double) * data->n );
	}

public:
	bool write( ssmTimeT time = gettimeSSM() )
	{
		if( !isOpen(  ) )
			return false;
		int tid;
		tid = writeSSMP( ssmId, &data, time, SSMParticleEvaluation::_ssmWrite, NULL );
		if(tid >= 0){timeId = tid; this->time = time; return true;}
		return false;
	}

	/// @brief tidを指定して読み込み
	/// @return 正しく読み込めたときtrueを返す
	bool read(int timeId = -1)
	{
		if( !isOpen(  ) )
			return false;
		int tid;
		tid = readSSMP( ssmId, &data, &time, timeId, SSMParticleEvaluation::_ssmRead, NULL );
		if(tid >= 0){this->timeId = tid; return true;}
		return false;
	}

	/**
	 * @brief 時間指定読み込み
	 * @return 正しく読み込めたときtrueを返す
	 *
	 * @see readSSM_time, read
	 */
	bool readTime( ssmTimeT time )
	{
		if( !isOpen(  ) )
		{
			return false;
		}
		SSM_tid tid = readSSMP_time( ssmId, mData, time, &( this->time ),
				SSMParticleEvaluation::_ssmRead, NULL );
		if( tid >= 0 )
		{
			timeId = tid;
			return true;
		}
		return false;
	}


};


#endif
