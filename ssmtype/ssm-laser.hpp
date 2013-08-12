/*
 * ssm-laser.hpp
 *
 *  Created on: 2010/07/14
 *      Author: shikina
 *      ssm-lidar.hppがちょっと整理が付かないので、新しく作り直し
 */



#ifndef SSMLASER_HPP_
#define SSMLASER_HPP_

#if 1
/*
 * ssm-laser.hpp
 *
 *  Created on: 2011/06/02
 *      Author: shikina
 */

#include <iostream>
#include <stdexcept>
#include <cstring>
#include <sys/uio.h>

#include <stdint.h>

#include <ssm.hpp>

#include "gnd-matrix-base.hpp"

class SSMScanPoint2D;

// ---> constant definition
namespace ssm {
	namespace laser
	{
		enum
		{
			STATUS_OK = 0,							///< 正常値

			STATUS_WARNING = 0x70000000,			///< これ以降は警告
			STATUS_NO_REFLECTION,					///< 受光無し
			STATUS_DIST_WARNING,					///< 距離が遠すぎるかも
			STATUS_ANGLE_WARNING,					///< 角度がおかしいかも

			STATUS_ERROR = 0x80000000,				///< これ以降はエラー


		};
	};
}
// ---> constant  definition

// ---> type declaration
namespace ssm {

	/// 2次元極座標系測距点
	class MeasuredPoint2DPolar
	{
		// ---> constructor, destructor
	public:
		MeasuredPoint2DPolar();
		MeasuredPoint2DPolar( double th, double r, double intensity = -1 );

	public:
		double th;					///< 角度(rad)
		double r;					///< 距離(m)
		double intensity;			///< 受光強度
		uint32_t status;			///< ステータス

		// ---> setter
	public:
		void set( double th, double r, double intensity = -1 );

		// ---> is
	public:
		bool isWarning(  ) const;
		bool isError(  ) const;
	};




	class ScanPoint2D
	{
	// ---> constructor,destructor
	public:
		ScanPoint2D(  );
		~ScanPoint2D(  );

	// ---> allocator
	public:
		bool alloc( uint32_t numPoints );

	// ---> getter, setter, indexer
	public:
		/// getter
		double timeStamp(  ) const;
		/// setter
		void timeStamp( double timeStamp );
		/// getter
		uint32_t numPoints(  ) const;


		size_t _ssmSize(  )
		{
			return sizeof( ScanPoint2D ) + sizeof( MeasuredPoint2DPolar ) * _numPoints;
		}

	// ---> operator
	public:
		MeasuredPoint2DPolar &operator[]( const uint32_t &index );
		ScanPoint2D &operator=( ScanPoint2D &scan );
		friend std::ostream& operator<< ( std::ostream &os, const ScanPoint2D &scan );

	// ---> static function
	public:
		static void _ssmWrite( void *assmp, const void *adata, void *userData );

		static void _ssmRead( const void *assmp, void *adata, void *auserData );
		static int binarysize(ScanPoint2D *scan);
		static int tobinary( ScanPoint2D *scan, char *buf );
		static int frombinary( ScanPoint2D *scan, char *buf );

		static int write( int fd, ScanPoint2D* scan );
		static int read( int fd, ScanPoint2D* scan );
		static int read_udp( int fd, ScanPoint2D* scan );

	private:
		double _timeStamp;							///< タイムスタンプ
		uint32_t _numPoints;						///< 点数
		MeasuredPoint2DPolar *_points;				///< データ

	};

	class ScanPoint2DProperty
	{
	public:
		const static size_t LENGTH_MAX = 128;

		uint32_t numPoints;							///< 計測個数
		double distMin, distMax;					///< 計測可能距離(m)
		double angMin, angMax;						///< 計測可能角度(rad)
		double angResolution;						///< 分解能（１ステップあたりの角度）(rad)
		double cycle;								///< 周波数(s)
		struct
		{
			char firmware[LENGTH_MAX];				///< ファームウェアバージョン
			char product[LENGTH_MAX];				///< センサモデル
			char protocol[LENGTH_MAX];				///< 通信プロトコル
			char id[LENGTH_MAX];					///< センサID
			char vendor[LENGTH_MAX];				///< 製造会社
		}sensorInfo;

		//	class Position3D
		//	{
		//	public:
		//		double Tx,Ty,Tz;								///< ロボットの中心からの位置(m)
		//		double Rx,Ry,Rz;								///< 取り付け角度(rad)
		//	}pos;

		gnd::matrix::fixed<4,4> coordm;

		friend std::ostream& operator<< ( std::ostream &os, const ScanPoint2DProperty &prop )
		{
			os
			<< "point      : " << prop.numPoints << std::endl
			<< std::endl
			<< "distance   : " << prop.distMin << " to " << prop.distMax << std::endl
			<< "angle      : " << prop.angMin << " to " << prop.angMax << std::endl
			<< "resolution : " << prop.angResolution << std::endl
			<< "cycle      : " << prop.cycle << std::endl
			<< std::endl
			<< "firmware   : " << prop.sensorInfo.firmware <<std::endl
			<< "product    : " << prop.sensorInfo.product <<std::endl
			<< "protocol   : " << prop.sensorInfo.protocol <<std::endl
			<< "id         : " << prop.sensorInfo.id <<std::endl
			<< "vendor     : " << prop.sensorInfo.vendor <<std::endl
			<< std::endl
			//			<< "pos(trans) : " << prop.pos.Tx << " " << prop.pos.Tx << " " << prop.pos.Tz << std::endl
			//			<< "pos(rot)   : " << prop.pos.Rx << " " << prop.pos.Rx << " " << prop.pos.Rz << std::endl
			;
			return os;
		}
	};


	/// ビームデータ
	class Beam3D
	{
	public:
		union Point3D
		{
			struct
			{
				double x,y,z;						/// 位置(m)
			};
			double vec[3];
		};

		Point3D origin;								///< センサ位置
		Point3D reflect;							///< 反射位置
		double intensity;							///< 反射強度
		uint32_t status;								///< ステータス(m)


		/// エラーチェック
		bool isWarning(  ) const { return ( status >= laser::STATUS_WARNING ); }
		bool isError(  ) const { return ( status >= laser::STATUS_ERROR ); }

		friend std::ostream& operator<< ( std::ostream &os, const Beam3D::Point3D &p )
		{
			os << p.x << ' ' << p.y << ' ' << p.z;
			return os;
		}

		friend std::ostream& operator<< ( std::ostream &os, const Beam3D &beam )
		{
			os << beam.origin << ' ' << beam.reflect << ' ' << beam.intensity;
			return os;
		}

	};



	/// ３次元測域センサデータ
	class SOKUIKIData3D
	{
	public:

		SOKUIKIData3D(  )
		{
			_numPoints = 0;
			_beams = NULL;
		}
		~SOKUIKIData3D(  )
		{
			delete [] _beams;
		}

		// alloc
		bool alloc( uint32_t numPoints )
		{
			_beams = new Beam3D[numPoints];
			_numPoints = numPoints;
			return true;
		}

		/// getter
		double timeStamp(  ) const
		{ return _timeStamp; }

		/// setter
		void timeStamp( double timeStamp )
		{
			_timeStamp = timeStamp;
		}

		/// getter
		uint32_t numPoints(  ) const
		{
			return _numPoints;
		}

		Beam3D &operator[]( const uint32_t &index )
		{
			return _beams[index];
		}

		SOKUIKIData3D &operator=( SOKUIKIData3D &data )
		{
			if( _numPoints == 0 )
			{
				_beams = new Beam3D[data._numPoints];
			}
			else if( data._numPoints != _numPoints )
			{
				throw std::invalid_argument( "SOKUIKIData3D::operator=() : numPoints must be equal." );
			}
			_timeStamp = data._timeStamp;
			_numPoints = data._numPoints;
			memcpy( _beams, data._beams,  sizeof( Beam3D ) * _numPoints );
			return *this;
		}

		friend std::ostream& operator<< ( std::ostream &os, const SOKUIKIData3D &data )
		{
			Beam3D b;
			for( uint32_t i = 0; i < data.numPoints(  ); i++ )
			{
				b = data._beams[i];
				if( !b.isWarning(  ) )
					os << b << std::endl;
			}
			return os;
		}

		size_t _ssmSize(  )
		{
			return sizeof( SOKUIKIData3D ) + sizeof( Beam3D ) * _numPoints;
		}

		static void _ssmWrite( void *assmp, const void *adata, void *userData )
		{
			SOKUIKIData3D *ssmp = static_cast<SOKUIKIData3D *>( assmp );
			const SOKUIKIData3D *data = static_cast<const SOKUIKIData3D *>( adata );

			ssmp->_timeStamp = data->_timeStamp;
			ssmp->_numPoints = data->_numPoints;
			memcpy( static_cast<char *>(assmp) + sizeof( SOKUIKIData3D ), data->_beams, sizeof( Beam3D ) * data->_numPoints );
		}

		static void _ssmRead( const void *assmp, void *adata, void *auserData )
		{
			const SOKUIKIData3D *ssmp = static_cast<const SOKUIKIData3D *>( assmp );
			SOKUIKIData3D *data = static_cast<SOKUIKIData3D *>( adata );
			if( data->_numPoints == 0 )
			{
				data->_beams = new Beam3D[ssmp->_numPoints];
			}
			else if( data->_numPoints != ssmp->_numPoints )
			{
				throw std::invalid_argument( "SOKUIKIData3D::operator=() : numPoints must be equal." );
			}

			data->_timeStamp = ssmp->_timeStamp;
			data->_numPoints = ssmp->_numPoints;
			memcpy( data->_beams, static_cast<const char *>(assmp) + sizeof( SOKUIKIData3D ), sizeof( Beam3D ) * ssmp->_numPoints );
		}


	private:
		uint32_t _numPoints;
		double _timeStamp;
		Beam3D *_beams;
	};



	typedef ScanPoint2DProperty SOKUIKIData3DProperty;

} // namespace ssm
// ---> type declaration



// ---> function definition
namespace ssm {
	// ---> MeasuredPoint2DPolar fucntion
	inline
	MeasuredPoint2DPolar::MeasuredPoint2DPolar()
	{

	}

	inline
	MeasuredPoint2DPolar::MeasuredPoint2DPolar( double th, double r, double intensity )
	{
		set( th, r, intensity );
	}

	inline
	void MeasuredPoint2DPolar::set( double th, double r, double intensity )
	{
		this->th = th;
		this->r = r;
		this->intensity = intensity;
	}

	inline
	bool MeasuredPoint2DPolar::isWarning(  ) const
	{
		return ( status >= laser::STATUS_WARNING );
	}

	inline
	bool MeasuredPoint2DPolar::isError(  ) const
	{
		return ( status >= laser::STATUS_ERROR );
	}


	/// 出力
	inline std::ostream& operator<< (std::ostream &os, const MeasuredPoint2DPolar &p)
	{
		os << p.th << ' ' << p.r << ' ' << p.intensity;
		return os;
	}
	// <--- MeasuredPoint2DPolar fucntion




	// ----> ScanPoint2D function
	inline
	ScanPoint2D::ScanPoint2D(  )
	{
		_numPoints = 0;
		_points = NULL;
	}

	inline
	ScanPoint2D::~ScanPoint2D(  )
	{
		delete [] _points;
	}

	inline
	bool ScanPoint2D::alloc( uint32_t numPoints )
	{
		_points = new MeasuredPoint2DPolar[numPoints];
		_numPoints = numPoints;
		return true;
	}

	inline
	double ScanPoint2D::timeStamp(  ) const
	{ return _timeStamp; }

	/// setter
	inline
	void ScanPoint2D::timeStamp( double timeStamp )
	{
		_timeStamp = timeStamp;
	}

	/// getter
	inline
	uint32_t ScanPoint2D::numPoints(  ) const
	{
		return _numPoints;
	}

	inline
	MeasuredPoint2DPolar &ScanPoint2D::operator[]( const uint32_t &index )
	{
		return _points[index];
	}

	inline
	ScanPoint2D &ScanPoint2D::operator=( ScanPoint2D &scan )
	{
		if( _numPoints == 0 )
		{
			_points = new MeasuredPoint2DPolar[scan._numPoints];
		}
		else if( scan._numPoints != _numPoints )
		{
			throw std::invalid_argument( "ScanPoint2D::operator=() : numPoints must be equal." );
		}
		_timeStamp = scan._timeStamp;
		_numPoints = scan._numPoints;
		memcpy( _points, scan._points, sizeof( MeasuredPoint2DPolar ) * _numPoints );
		return *this;
	}

	inline
	std::ostream& operator<< ( std::ostream &os, const ScanPoint2D &scan )
	{
		MeasuredPoint2DPolar p;
		for( uint32_t i = 0; i < scan.numPoints(  ); i++ )
		{
			p = scan._points[i];
			if( !p.isWarning(  ) );
			os << p << std::endl;
		}
		return os;
	}

	inline
	void ScanPoint2D::_ssmWrite( void *assmp, const void *adata, void *userData )
	{
		ScanPoint2D *ssmp = static_cast<ScanPoint2D *>( assmp );
		const ScanPoint2D *data = static_cast<const ScanPoint2D *>( adata );

		ssmp->_timeStamp = data->_timeStamp;
		ssmp->_numPoints = data->_numPoints;
		memcpy( static_cast<char *>(assmp) + sizeof( ScanPoint2D ), data->_points, sizeof( MeasuredPoint2DPolar ) * data->_numPoints );
	}

	inline
	void ScanPoint2D::_ssmRead( const void *assmp, void *adata, void *auserData )
	{
		const ScanPoint2D *ssmp = static_cast<const ScanPoint2D *>( assmp );
		ScanPoint2D *data = static_cast<ScanPoint2D *>( adata );
		if( data->_numPoints == 0 )
		{
			data->_points = new MeasuredPoint2DPolar[ssmp->_numPoints];
		}
		else if( data->_numPoints != ssmp->_numPoints )
		{
			throw std::invalid_argument( "ScanPoint2D::operator=() : numPoints must be equal." );
		}

		data->_timeStamp = ssmp->_timeStamp;
		data->_numPoints = ssmp->_numPoints;
		memcpy( data->_points, static_cast<const char *>(assmp) + sizeof( ScanPoint2D ), sizeof( MeasuredPoint2DPolar ) * ssmp->_numPoints );
	}

	inline
	int ScanPoint2D::binarysize(ScanPoint2D *scan){
		return sizeof(scan->_timeStamp) + sizeof(scan->_numPoints) + sizeof( MeasuredPoint2DPolar ) * scan->_numPoints;
	}

	inline
	int ScanPoint2D::tobinary( ScanPoint2D *scan, char *buf )
	{
		int offset = 0;

		::memcpy(buf + offset, &scan->_timeStamp, sizeof(scan->_timeStamp));
		offset += sizeof(scan->_timeStamp);

		::memcpy(buf + offset, &scan->_numPoints, sizeof(scan->_numPoints));
		offset += sizeof(scan->_numPoints);

		::memcpy(buf + offset, scan->_points, sizeof( MeasuredPoint2DPolar ) * scan->_numPoints);
		offset += sizeof( MeasuredPoint2DPolar ) * scan->_numPoints;
		return offset;
	}

	inline
	int ScanPoint2D::frombinary( ScanPoint2D *scan, char *buf )
	{
		int offset = 0;

		::memcpy(&scan->_timeStamp, buf + offset, sizeof(scan->_timeStamp));
		offset += sizeof(scan->_timeStamp);

		::memcpy(&scan->_numPoints, buf + offset, sizeof(scan->_numPoints));
		offset += sizeof(scan->_numPoints);

		::memcpy(scan->_points, buf + offset, sizeof( MeasuredPoint2DPolar ) * scan->_numPoints);
		offset += sizeof( MeasuredPoint2DPolar ) * scan->_numPoints;
		return offset;
	}

	inline
	int ScanPoint2D::write( int fd, ScanPoint2D* scan ){
		struct iovec iov[3];

		iov[0].iov_base = &scan->_timeStamp;
		iov[0].iov_len = sizeof(scan->_timeStamp);
		iov[1].iov_base = &scan->_numPoints;
		iov[1].iov_len = sizeof(scan->_numPoints);
		iov[2].iov_base = scan->_points;
		iov[2].iov_len = sizeof( MeasuredPoint2DPolar ) * scan->_numPoints;
		return writev(fd, iov, 3);
	}

	inline
	int ScanPoint2D::read( int fd, ScanPoint2D* scan ){
		uint32_t n;
		struct iovec iov[2];

		iov[0].iov_base = &scan->_timeStamp;
		iov[0].iov_len = sizeof(scan->_timeStamp);
		iov[1].iov_base = &n;
		iov[1].iov_len = sizeof(scan->_numPoints);
		if( readv(fd, iov, 2) < 0 )	return -1;
		if( scan->_numPoints != n ) {
			if( !scan->_points ) delete[] scan->_points;
			scan->alloc(n);
		}
		if( (::read(fd, scan->_points, sizeof( MeasuredPoint2DPolar ) * n)) < 0 ){
			return -1;
		}
		return 0;
	}

	inline
	int ScanPoint2D::read_udp( int fd, ScanPoint2D* scan ){
		if(!scan->_numPoints) {
			struct iovec iov[2];
			uint32_t n;

			iov[0].iov_base = &scan->_timeStamp;
			iov[0].iov_len = sizeof(scan->_timeStamp);
			iov[1].iov_base = &n;
			iov[1].iov_len = sizeof(scan->_numPoints);

			if( readv(fd, iov, 2) < 0 )	return -1;
			scan->alloc(n);
			return 1;
		}
		else {
			struct iovec iov[3];
			uint32_t n;

			iov[0].iov_base = &scan->_timeStamp;
			iov[0].iov_len = sizeof(scan->_timeStamp);
			iov[1].iov_base = &n;
			iov[1].iov_len = sizeof(scan->_numPoints);
			iov[2].iov_base = scan->_points;
			iov[2].iov_len = sizeof(MeasuredPoint2DPolar) * scan->_numPoints;

			if( readv(fd, iov, 3) < 0 )	return -1;
			return 0;
		}

	}
	// <---- ScanPoint2D function


} // namespace ssm
// <--- function definition





//typedef SSMApi<ssm::ScanPoint2D, ssm::ScanPoint2DProperty> SSMScanPoint2D;

class SSMScanPoint2D : public SSMApi<ssm::ScanPoint2D, ssm::ScanPoint2DProperty>
{

public:
	SSMScanPoint2D(){}
	SSMScanPoint2D( const char *streamName, int streamId = 0 ) : SSMApi<ssm::ScanPoint2D, ssm::ScanPoint2DProperty>( streamName, streamId ){}
	/// @brief timeを指定して書き込み
	/// @param time[in] 時間。指定しないときは現在時刻を書き込み
	/// @return 正しく書き込めたときtrueを返す
	bool write( ssmTimeT time = gettimeSSM() )
	{
		if( !isOpen(  ) )
			return false;
		int tid;
		tid = writeSSMP( ssmId, &data, time, ssm::ScanPoint2D::_ssmWrite, NULL );
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
		tid = readSSMP( ssmId, &data, &time, timeId, ssm::ScanPoint2D::_ssmRead, NULL );
		if(tid >= 0){this->timeId = tid; return true;}
		return false;
	}

	bool readTime( ssmTimeT time )
	{
		if( !isOpen(  ) )
			return false;
		SSM_tid tid = readSSMP_time( ssmId, &data, time, &( this->time ), ssm::ScanPoint2D::_ssmRead, NULL );
		if( tid >= 0 )
		{
			timeId = tid;
			return true;
		}
		return false;
	}

	///
	size_t sharedSize(  )
	{
		return data._ssmSize(  );
	}
};

class SSMSOKUIKIData3D : public SSMApi<ssm::SOKUIKIData3D, ssm::SOKUIKIData3DProperty>
{

public:
	SSMSOKUIKIData3D(){}
	SSMSOKUIKIData3D( const char *streamName, int streamId = 0 ) : SSMApi<ssm::SOKUIKIData3D, ssm::SOKUIKIData3DProperty>( streamName, streamId ){}
	/// @brief timeを指定して書き込み
	/// @param time[in] 時間。指定しないときは現在時刻を書き込み
	/// @return 正しく書き込めたときtrueを返す
	bool write( ssmTimeT time = gettimeSSM() )
	{
		if( !isOpen(  ) )
			return false;
		int tid;
		tid = writeSSMP( ssmId, &data, time, ssm::SOKUIKIData3D::_ssmWrite, NULL );
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
		tid = readSSMP( ssmId, &data, &time, timeId, ssm::SOKUIKIData3D::_ssmRead, NULL );
		if(tid >= 0){this->timeId = tid; return true;}
		return false;
	}

	bool readTime( ssmTimeT time )
	{
		SSM_tid tid = readSSMP_time( ssmId, &data, time, &( this->time ), ssm::SOKUIKIData3D::_ssmRead, NULL );
		if( tid >= 0 )
		{
			timeId = tid;
			return true;
		}
		return false;
	}
	///
	size_t sharedSize(  )
	{
		return data._ssmSize(  );
	}
};



//static const char は微妙だった気がするので、define
#define SSM_NAME_SCAN_POINT_2D "scan_data2d"
#define SSM_NAME_SOKUIKI_3D_FS "sokuiki_fs"
#define SSM_NAME_SOKUIKI_3D_BS "sokuiki_bs"

#else

#include <ssm.hpp>
#include <iostream>

namespace ssm
{


	//static const int MAX_SCAN_POINTS = 1200;
#ifndef MAX_SCAN_POINTS
#define MAX_SCAN_POINTS 1200
#endif

	// 2次元
	class OnePoint2D
	{
		void init(double th,int d)
		{
			this->th = th;
			this->d = d;
		}
	public:
		double th;
		int d;
		bool isError(){return ( d < 20 );}
		OnePoint2D(){}
		OnePoint2D(double th,int d){init(th,d);}
	};
	inline std::ostream& operator<< (std::ostream &os, const OnePoint2D &p)
	{
		os << p.d << ' ' << p.th;
		return os;
	}

	class ScanPoint2D
	{
	public:
		int minstep, maxstep, skip, numPoints;
		double res;
		OnePoint2D point[MAX_SCAN_POINTS];
		double timestamp;
		int reflect[MAX_SCAN_POINTS]; //これまでのLOGデータが再生できるように、ここに宣言
	};
	inline std::ostream& operator<< (std::ostream &os, const ScanPoint2D &p)
	{
		for(int i=0; i<p.numPoints; i++)
		{
			if( p.point[i].d > 20 && p.point[i].d < 4096 )
				os << p.point[i] << ' ' << p.reflect[i] << ' ' << p.point[i].d << std::endl;
			else
				os << 0 << ' ' << p.point[i].th << ' ' << p.reflect[i] << ' ' << 4096 << std::endl;
		}
		return os;
	}

	class ScanPoint2DProperty
	{
	public:
		int dist_min, dist_max;
		int steps_total, steps_min, steps_max;
		int rpm;
		char serial_number[8];
	};


	// 3次元
	class OnePoint3D
	{
	public:
		double x,y,z;
		double reflect; // 0 ~ 1の範囲
		OnePoint3D(){}
		OnePoint3D(double x,double y, double z, double reflect):x(x),y(y),z(z),reflect(reflect){}
	};
	inline std::ostream& operator<< (std::ostream &os, const OnePoint3D &p)
	{
		os << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.reflect;
		return os;
	}

	class ScanPoint3D
	{
	public:
		int numPoints;
		double timestamp;
		OnePoint3D point[MAX_SCAN_POINTS];
		OnePoint2D raw[MAX_SCAN_POINTS];
	};
	inline std::ostream& operator<< (std::ostream &os, const ScanPoint3D &p)
	{
		for(int i=0; i<p.numPoints; i++)
		{
			if(p.raw[i].d < 20){continue;}
			os << p.point[i] << std::endl;
		}
		return os;
	}

	// ScanPoint2DとScanPoint3Dのプロパティは一緒だが、これからのことを考えて一応名前を変えておく
	typedef ScanPoint2DProperty ScanPoint3DProperty;


	class ScanLine2D
	{
	public:
		double x1,y1,x2,y2; // 直線
		double zmin, zmax; // 検出するときの高さの範囲
		double confidence; // 信頼度を入れたい。とりあえずハフ変換の投票数
		bool isMatching;
	};
	inline std::ostream& operator<< (std::ostream &os, const ScanLine2D &l)
	{
		os << l.x1 << ' ' << l.y1 << ' ' << l.x2 << ' ' << l.y2 << ' '
				<< l.zmin << ' ' << l.zmax << ' ' << l.confidence << ' ' << l.isMatching;
		return os;
	}


	class ScanLine2DLandmark
	{
	public:
		double x1,y1,x2,y2; // 直線
		double zmin, zmax; // 検出するときの高さの範囲

		bool set( double x1, double y1, double x2, double y2, double zmin, double zmax )
		{
			this->x1 = x1;
			this->y1 = y1;
			this->x2 = x2;
			this->y2 = y2;
			this->zmin = zmin;
			this->zmax = zmax;
			return true;
		}
	};

	//#define SCAN_LINE_MAX_NUM 200
	static const int SCAN_LINE_MAX_NUM = 200;
	class ScanLine2DProperty
	{
	public:
		int numLines;//線の個数
		ScanLine2DLandmark line[SCAN_LINE_MAX_NUM]; //線
		ScanLine2DProperty(){init();}

		void init()
		{
			numLines = 0;
		}
		bool set( double x1, double y1, double x2, double y2, double zmin, double zmax )
		{
			if( numLines >= SCAN_LINE_MAX_NUM - 1 )
				return false;
			numLines++;
			return line[numLines].set(x1, y1, x2, y2, zmin,zmax);
		}
	};

} // namespace ssm

typedef SSMApi<ssm::ScanPoint2D, ssm::ScanPoint2DProperty> SSMScanPoint2D;
typedef SSMApi<ssm::ScanPoint3D, ssm::ScanPoint3DProperty> SSMScanPoint3D;
typedef SSMApi<ssm::ScanLine2D, ssm::ScanLine2DProperty > SSMScanLine2D;

//static const char は微妙だった気がするので、define
#define SSM_NAME_URG_RAW "urg_scip2.0"
#define SSM_NAME_URG_FS "urg_fs2"
#define SSM_NAME_URG_BS "urg_bs2"

#define SSM_NAME_URG_LINE_BS "urg_line_bs2"

// 木の位置, 半径, 検出及びマッチングフラグを格納する構造体 (ssm-lidar.hppより引き継ぎ) YTS
typedef struct {
	double x , y; 		// "spur_global" sid=1 の座標系での木の位置
	double	radius;		// 木の半径
	int matching;	// "0"のときは木を検出した。 "1"のときは木のマッチングに成功した。
} TreeParam;

// 路面検出
#define SSM_NAME_URG_ROAD_OBJ_BS "urg_road_obj_bs"
#define SSM_NAME_URG_ROAD_OBJ_FS "urg_road_obj_fs"


#endif

#endif /* SSMLASER_HPP_ */
