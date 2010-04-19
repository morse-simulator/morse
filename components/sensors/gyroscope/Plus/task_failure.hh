#ifndef _TASK_FAILURE_HH_
#define _TASK_FAILURE_HH_

#include <stdexcept>	
#include <string>
#include <sstream>

class TaskException :  public std::runtime_error {
	protected:
		std::string taskName;

	public:
		TaskException(const std::string & _taskName) : std::runtime_error("TaskException"),
												   taskName(_taskName) {}; 
		virtual ~TaskException() throw () {};
		virtual const char* what() const throw() = 0;
};

class TaskExecutionFailureException : public TaskException {
	private:
		std::string methodName;
		std::string taskError;

	public:
		TaskExecutionFailureException(const std::string & _taskName, const std::string & _methodName,
									  const std::string & _taskError) :
			TaskException(_taskName), methodName(_methodName), taskError(_taskError) {};
		virtual ~TaskExecutionFailureException() throw () {};
		virtual const char * what() const throw() {
			std::ostringstream oss;
			oss << "Failure in " << taskName << "::" << methodName << " : " << taskError;
			return oss.str().c_str();
		};

		virtual const std::string& errorName() const throw() {
			return taskError;
		};
};

#endif /* _TASK_FAILURE_HH_ */
