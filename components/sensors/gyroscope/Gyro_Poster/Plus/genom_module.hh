#ifndef _GENOM_MODULE_HH_
#define _GENOM_MODULE_HH_


#include <string>
#include <list>
#include <algorithm>

#include <cassert>
#include <cstdio>

#include <csLib.h>

#include <task_failure.hh>

#define define_method_exec(modName, methodName) \
	class methodName : public GenomExecActivity { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomExecActivity(mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

#define define_method_exec_input(modName, methodName, input) \
	class methodName : public GenomExecInputActivity<input> { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomExecInputActivity<input>(mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

#define define_method_exec_output(modName, methodName, output) \
	class methodName : public GenomExecOutputActivity<output> { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomExecOutputActivity<output>(mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

#define define_method_exec_input_output(modName, methodName, input, output) \
	class methodName : public GenomExecInputOutputActivity<input, output> { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomExecInputOutputActivity<input, output> \
								  (mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

#define define_method_ctrl(modName, methodName) \
	class methodName : public GenomCtrlActivity { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomCtrlActivity(mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

#define define_method_ctrl_input(modName, methodName, input) \
	class methodName : public GenomCtrlInputActivity<input> { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomCtrlInputActivity<input>        \
								  (mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

#define define_method_ctrl_input_output(modName, methodName, input, output) \
	class methodName : public GenomCtrlInputOutputActivity<input, output> { \
		public: \
			methodName(GenomModule* mod) :          \
				GenomCtrlInputOutputActivity<input, output> \
								  (mod,				 \
							      #methodName,       \
							      modName##AbortRqstSend, \
								  modName##methodName##RqstSend, \
								  modName##methodName##ReplyRcv) {}; \
				virtual ~methodName () {}; \
	}; 

class GenomActivity;

struct Pocolibs {
	/*
	 * This method must be called on each thread where you want to use a GenomModule
	 * Each GenomModule is directly linked with its internal mbox so you can't
	 * execute a method from a GenomModule in another thread.
	 */
	static void prepare(const std::string & name, int number_request)
	{
		csMboxInit(const_cast<char*> (name.c_str()), number_request, number_request);
	}
};

class GenomModule {
	private:
		std::list<GenomActivity*> activities;

		void addActivity(GenomActivity* a) { activities.push_back(a); };
		void removeActivity(GenomActivity* a) 
		{ 
			std::list<GenomActivity*>::iterator it;
			it = std::find(activities.begin(), activities.end(), a);
			if (it != activities.end())
				activities.erase(it);
		}

	protected:
		CLIENT_ID id;
		std::string module_name;
		static void string_to_buffer(const std::string & str, char* buffer, size_t size)
		{
			assert(buffer != NULL);
			snprintf(buffer, size, "%s", str.c_str());
		};

	public:
		GenomModule(const std::string &name) : module_name(name) {};
		virtual ~GenomModule() {};

		CLIENT_ID getId() { return id; };

		friend class GenomActivity;
};

class GenomActivity {
	protected:
		GenomModule* mod;
		std::string f_name;
		int rqstId;
		int activity;
		int report;

		int (*f_abort) (CLIENT_ID, int*, int*, int);

		std::string errorMsg() const {
			char buf[1024];
			h2getErrMsg(report, buf, sizeof(buf));
			return std::string(buf);
		};

		void checkSuccess() 
		{
			if (report != OK)
				throw TaskExecutionFailureException(mod->module_name, f_name, errorMsg());
		}

	public:
		GenomActivity(GenomModule* _mod, const std::string& _fname, 
				int(*_f)(CLIENT_ID, int*, int*, int)) :
		  	mod(_mod) , f_name(_fname), rqstId(-1), f_abort(_f)
		{
			mod->addActivity(this);
		};

		virtual ~GenomActivity() 
		{
		   	abort();
			mod->removeActivity(this);
	   	};

		void abort()
	   	{
			if (rqstId != -1) {
				// Call Abort method
				int tmpRqstId;
				f_abort(mod->getId(), &tmpRqstId, &rqstId, 0);
			}

			rqstId = -1;
		}
};

class GenomExecActivity : public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, int);
		int (*f_wait) (CLIENT_ID, int, int, int*, int*);

	public:
		GenomExecActivity(GenomModule* mod,
				const std::string& fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, int),
				int (*_f_wait)(CLIENT_ID, int, int, int*, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};
				   
		virtual ~GenomExecActivity() {};

		void call()
		{
			if (rqstId == -1)  
				f_call(mod->getId(), &rqstId, 0);
		}

		void wait()
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY, &activity, &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait()
		{
			call();
			wait();
		}

		bool isFinished() 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, &activity, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};

template <typename T_input>
class GenomExecInputActivity : public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, T_input*, int);
		int (*f_wait) (CLIENT_ID, int, int, int*, int*);

	public:
		GenomExecInputActivity(GenomModule* mod,
				const std::string&fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, T_input*, int),
				int (*_f_wait)(CLIENT_ID, int, int, int*, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};

		virtual ~GenomExecInputActivity() {};

		void call(T_input* input)
		{
			if (rqstId == -1) 
				f_call(mod->getId(), &rqstId, input, 0);
			// Throw an exception if we try to call twice ?
		}

		void wait()
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY, &activity, &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait(T_input* input)
		{
			call(input);
			wait();
		}

		bool isFinished() 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, &activity, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};

template <typename T_output>
class GenomExecOutputActivity: public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, int);
		int (*f_wait) (CLIENT_ID, int, int, T_output*, int*, int*);

	public:
		GenomExecOutputActivity(GenomModule* mod,
				const std::string &fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, int),
				int (*_f_wait)(CLIENT_ID, int, int, T_output*, int*, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};

		virtual ~GenomExecOutputActivity() {};

		void call()
		{
			if (rqstId == -1) 
				f_call(mod->getId(), &rqstId, 0);
			// Throw an exception if we try to call twice ?
		}

		void wait(T_output* output)
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY, output, &activity, &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait(T_output* output)
		{
			call();
			wait(output);
		}

		bool isFinished(T_output* output) 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, output, &activity, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};

template <typename T_input, typename T_output>
class GenomExecInputOutputActivity : public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, T_input*, int);
		int (*f_wait) (CLIENT_ID, int, int, T_output*, int*, int*);

	public:
		GenomExecInputOutputActivity(GenomModule* mod,
				const std::string & fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, T_input*, int),
				int (*_f_wait)(CLIENT_ID, int, int, T_output*, int*, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};

		virtual ~GenomExecInputOutputActivity() {};

		void call(T_input* input)
		{
			if (rqstId == -1) 
				f_call(mod->getId(), &rqstId, input, 0);
			// Throw an exception if we try to call twice ?
		}

		void wait(T_output* output)
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY, output, &activity, &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait(T_input* input, T_output* output)
		{
			call(input);
			wait(output);
		}

		bool isFinished(T_output* output) 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, output, &activity, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};

class GenomCtrlActivity : public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, int);
		int (*f_wait) (CLIENT_ID, int, int, int*);

	public:
		GenomCtrlActivity(GenomModule* mod,
				const std::string &fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, int),
				int (*_f_wait)(CLIENT_ID, int, int, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};
				   
		virtual ~GenomCtrlActivity() {};

		void call()
		{
			if (rqstId == -1) 
				f_call(mod->getId(), &rqstId, 0);
			// Throw an exception if we try to call twice ?
		}

		void wait()
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY,  &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait()
		{
			call();
			wait();
		}

		bool isFinished() 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};

template <typename T_input>
class GenomCtrlInputActivity : public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, T_input*, int);
		int (*f_wait) (CLIENT_ID, int, int, int*);

	public:
		GenomCtrlInputActivity(GenomModule* mod, 
				const std::string & fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, T_input*, int),
				int (*_f_wait)(CLIENT_ID, int, int, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};
				   
		virtual ~GenomCtrlInputActivity() {};

		void call(T_input* input)
		{
			if (rqstId == -1) 
				f_call(mod->getId(), &rqstId, input, 0);
			// Throw an exception if we try to call twice ?
		}

		void wait()
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY, &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait(T_input* input)
		{
			call(input);
			wait();
		}

		bool isFinished() 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};


template <typename T_input, typename T_output>
class GenomCtrlInputOutputActivity : public GenomActivity 
{
	protected:
		int (*f_call) (CLIENT_ID, int*, T_input*, int);
		int (*f_wait) (CLIENT_ID, int, int, T_output*, int*);

	public:
		GenomCtrlInputOutputActivity(GenomModule* mod,
				const std::string &fname,
				int (*_f_abort)(CLIENT_ID, int*, int*, int),
				int (*_f_call)(CLIENT_ID, int*, T_input*, int),
				int (*_f_wait)(CLIENT_ID, int, int, T_output*, int*)) :
					GenomActivity(mod, fname, _f_abort),
					f_call(_f_call),
					f_wait(_f_wait) {};
				   
		virtual ~GenomCtrlInputOutputActivity() {};

		void call(T_input* input)
		{
			if (rqstId == -1) 
				f_call(mod->getId(), &rqstId, input, 0);
			// Throw an exception if we try to call twice ?
		}

		void wait(T_output* output)
		{
			f_wait(mod->getId(), rqstId, BLOCK_ON_FINAL_REPLY,  output, &report);
			rqstId = -1;
			checkSuccess();
		}

		void callAndWait(T_input* input, T_output* output)
		{
			call(input);
			wait(output);
		}

		bool isFinished(T_output* output) 
		{
			if (rqstId == -1)
				return false;

			switch(f_wait(mod->getId(), rqstId, NO_BLOCK, output, &report)) {
				case FINAL_REPLY_OK:
					rqstId = -1;
					checkSuccess();
					return true;
				default:
					return false;
			}
		}
};

#endif /* _GENOM_MODULE_HH_ */
