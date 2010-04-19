#ifndef _POSTER_READ_HH_
#define _POSTER_READ_HH_

#include <string>

#include <posterLib.h>

#include <poster_exception.hh>
#include <poster_locker.hh>

template <typename T>
class PosterReader
{
	private:
		POSTER_ID id;
		const std::string posterName;

	public:
		PosterReader(const std::string & name) : posterName(name)
		{
			STATUS s = posterFind(const_cast<char*>(name.c_str()), &id);
			if (s == ERROR) 
				throw PosterFindException(posterName);
		}

		void get(T* buf, bool lock = false)
		{
			if (lock) 
				PosterLocker<true> locker(id, posterName);
				
			int err = posterRead(id, 0, static_cast<void*>(buf), sizeof(T));
			if (err != sizeof(T))
				throw PosterReadException<T> (posterName, err);
		}
};

#endif /* _POSTER_READ_HH_ */
