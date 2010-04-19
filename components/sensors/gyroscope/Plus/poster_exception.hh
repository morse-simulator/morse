#ifndef _POSTER_EXCEPTION_HH_
#define _POSTER_EXCEPTION_HH_

#include <string>
#include <sstream>
#include <stdexcept>

#include "errnoLib.h"
#include "h2errorLib.h"

class PosterException : public std::runtime_error {
	protected:
		std::string posterName;

	public:
		PosterException(const std::string& name) :
		   	std::runtime_error("PosterException"),
			posterName(name) {};
		virtual ~PosterException() throw() {};
		virtual const char* what () throw()  = 0;

		std::string errorMsg() const {
			char buf[1024];
			h2getErrMsg(errnoGet(), buf, sizeof(buf));
			return std::string(buf);
		};
};

class PosterFindException : public PosterException {
	public:
		PosterFindException(const std::string& name):
			PosterException(name) {};

		virtual const char* what() throw ()
		{
			std::ostringstream oss;
			oss << "Can't open " << posterName << " : " << errorMsg() << std::endl;
			return oss.str().c_str();
		}
};

class PosterCreateException : public PosterException {
	public:
		PosterCreateException(const std::string& name):
			PosterException(name) {};

		virtual const char* what() throw ()
		{
			std::ostringstream oss;
			oss << "Can't create " << posterName << " : " << errorMsg() << std::endl;
			return oss.str().c_str();
		}
};


template <typename T>
class PosterReadException : public PosterException {
	private:
		int read;

	public:
		PosterReadException(const std::string& name, int err):
			PosterException(name), read(err) {};

		virtual const char* what() throw ()
		{
			std::ostringstream oss;
			oss << "Read " << read << " bytes ";
			oss << " : expected " << sizeof(T) << " bytes "; 
			oss << " : " << errorMsg() << std::endl;
			return oss.str().c_str();
		}
};

template <typename T>
class PosterWriteException : public PosterException {
	private:
		int write;

	public:
		PosterWriteException(const std::string& name, int err):
			PosterException(name), write(err) {};

		virtual const char* what() throw ()
		{
			std::ostringstream oss;
			oss << "Write " << write << " bytes ";
			oss << " : expected " << sizeof(T) << " bytes ";
			oss << " : " << errorMsg() << std::endl;
			return oss.str().c_str();
		}
};

template <bool reader>
class PosterLockerException;

template <>
class PosterLockerException<true> : public PosterException {
	public:
		PosterLockerException(const std::string& name):
			PosterException(name) {};

		virtual const char* what() throw ()
		{
			std::ostringstream oss;
			oss << "Can't read lock " << posterName << " : " << errorMsg() << std::endl;
			return oss.str().c_str();
		}
};

template <>
class PosterLockerException<false> : public PosterException {
	public:
		PosterLockerException(const std::string& name):
			PosterException(name) {};

		virtual const char* what() throw ()
		{
			std::ostringstream oss;
			oss << "Can't write lock " << posterName << " : " << errorMsg() << std::endl;
			return oss.str().c_str();
		}
};
	
#endif /* _POSTER_EXCEPTION_HH_ */
