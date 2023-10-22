#include "cmdLineParser.h"

using namespace std;

//----------------------------------------------------------------------------------------------
void CmdLineParser::addFlag(std::string_view _tag, const FlagDelegate& _delegate)
{
	m_flags.insert(pair<string, FlagDelegate>(std::string("--") + _tag.data(), _delegate));
}

//----------------------------------------------------------------------------------------------
void CmdLineParser::addFlag(std::string_view _tag, bool& dst)
{
	addFlag(_tag, [&]() {
		dst = true;
		});
}

//----------------------------------------------------------------------------------------------
void CmdLineParser::addSimpleArgument(std::string_view _tag, const SimpleDelegate& _delegate)
{
	m_simpleArguments.insert(pair<string, SimpleDelegate>(std::string("--") + _tag.data(), _delegate));
}

//----------------------------------------------------------------------------------------------
void CmdLineParser::parse(int _argc, const char** _argv)
{
	for (int i = 0; i < _argc; ++i)
	{
		const char* arg = _argv[i];
		if (arg[0] != '-')
			continue;
		// Run simple arguments
		if (i < _argc - 1)
		{
			auto iter = m_simpleArguments.find(arg);
			if (iter != m_simpleArguments.end())
			{
				const char* nextArg = _argv[++i];
				iter->second(nextArg);
				continue;
			}
		}
		// Run flags
		auto iter = m_flags.find(arg);
		if (iter != m_flags.end())
		{
			iter->second();
		}
	}
}