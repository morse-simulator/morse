#ifndef _POSTER_WRITE_HH_
#define _POSTER_WRITE_HH_

#include <string>

#include <posterLib.h>

#include <poster_locker.hh>

template <typename T>
class PosterWriter
{
	private:
		POSTER_ID id;
		const std::string posterName;

	public:
		PosterWriter(const std::string & name) : posterName(name)
		{
			STATUS s = posterCreate(const_cast<char*>(name.c_str()), sizeof(T), &id);
			if (s == ERROR) 
				throw PosterCreateException(posterName);
		}

		void put(T* buf, bool lock = false)
		{
			if (lock) 
				PosterLocker<false> locker(id, posterName);
				
			int err = posterWrite(id, 0, static_cast<void*>(buf), sizeof(T));
			if (err != sizeof(T))
				throw PosterWriteException<T> (posterName, err);
		}

		~PosterWriter()
		{
			posterDelete(id);
		}
};

#endif /* _POSTER_WRITE_HH_ */
