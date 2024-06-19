#pragma once
#include <string>
#include <iostream>

class Log {
private:
	template<typename T, typename... Ts> static void Print(T arg, Ts... args) {
		std::cout << arg << " ";
		Log::Print(args...);
	}

	template<typename T> static void Print(T arg) {
		std::cout << arg << " ";
	}

public:
	template<typename... Ts> static void FormattedDebug(std::string class_name, std::string function_name, Ts... args) {
		std::cout << "[Debug Log] " + class_name + "::" + function_name + "(...)" + " - ";
		Log::Print(args...);
		std::cout << std::endl;
	}

	template<typename... Ts> static void Debug(Ts ...args) {
		std::cout << "[Debug Log] ";
		Log::Print(args...);
		std::cout << std::endl;
	}

	template<typename... Ts> static void Error(Ts ...args) {
		std::cout << "[Error] ";
		std::cout << "\a";
		if (sizeof...(args)) Log::Print(args...);
		else std::cout << std::endl;
		system("pause");
	}

	template<typename... Ts> static void Hand(Ts... args) {
		std::cout << "[Hand] ";
		if (sizeof...(args)) Log::Print(args...);
		else std::cout << std::endl;
	}

	template<typename... Ts> static void Print(Ts... args) {
		if (sizeof...(args)) Log::Print(args...);
		else std::cout << std::endl;
	}
};
