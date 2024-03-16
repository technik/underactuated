//--------------------------------------------------------------------------------------------------
// Revolution Engine
//--------------------------------------------------------------------------------------------------
// Copyright 2019 Carmelo J Fdez-Aguera
#pragma once

#include <functional>
#include <string>
#include <sstream>
#include <string_view>
#include <unordered_map>

class CmdLineParser
{
public:
	using FlagDelegate = std::function<void()>;
	using SimpleDelegate = std::function<void(const char*)>; // Will receive the first argument after the tag

	void addFlag(std::string_view tag, const FlagDelegate& _delegate);
	void addFlag(std::string_view tag, bool& dst);
	// Add an argument with just one parameter
	void addSimpleArgument(std::string_view tag, const SimpleDelegate& _delegate);

	template<class T>
	void addOption(std::string_view tag, T* dst);

	void parse(int argc, const char** argv);

private:
	std::unordered_map<std::string, FlagDelegate> m_flags;
	std::unordered_map<std::string, SimpleDelegate> m_simpleArguments;
};

template<class T>
inline void CmdLineParser::addOption(std::string_view tag, T* dst)
{
	addSimpleArgument(tag, [=](const char* value) {
		std::stringstream ss;
		ss << value;
		ss >> *dst;
		});
}

// Partial specialization for std::string
template<>
inline void CmdLineParser::addOption(std::string_view tag, std::string* dst)
{
	addSimpleArgument(tag, [=](const char* value) {
		*dst = value;
		});
}