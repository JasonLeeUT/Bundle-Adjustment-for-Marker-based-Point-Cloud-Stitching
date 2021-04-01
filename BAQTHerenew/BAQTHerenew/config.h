#ifndef CONFIG_H
#define CONFIG_H
#include "common_include.h"
namespace amnl 
{
	class Config
	{
	private:
		static std::shared_ptr<Config> config_;
		cv::FileStorage file_;

		Config() {} // private constructor makes a singleton
	public:
		~Config();  // close the file when deconstructing 

					// set a new config file 
		static void setParameterFile(const std::string& filename);

		// access the parameter values
		template< typename T >
		static T get(const std::string& key)
		{
			T temp;
			Config::config_->file_[key] >> temp;
			return temp;
		}
	};
}

#endif // CONFIG_H