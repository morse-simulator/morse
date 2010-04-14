#ifndef _POSTER_LOCKER_HH_
#define _POSTER_LOCKER_HH_

#include <posterLib.h>

template <bool reader> 
class PosterLocker; 

template <>
class PosterLocker<true>
{
	private:
		POSTER_ID id_;
		const std::string posterName;

	public:
		PosterLocker(const POSTER_ID id, const std::string name) :
		   	id_(id), posterName(name)
		{
			STATUS err = posterTake(id_, POSTER_READ);
			if (err == ERROR)
				throw PosterLockerException<true> (posterName);
		}

		~PosterLocker() 
		{
			posterGive(id_);
		}
};

template <>
class PosterLocker<false>
{
	private:
		POSTER_ID id_;
		const std::string posterName;

	public:
		PosterLocker(const POSTER_ID id, const std::string name) : 
			id_(id), posterName(name)
		{
			STATUS err = posterTake(id_, POSTER_WRITE);
			if (err == ERROR)
				throw PosterLockerException<false> (posterName);
		}

		~PosterLocker() 
		{
			posterGive(id_);
		}
};

#endif /* _POSTER_LOCKER_HH_ */
