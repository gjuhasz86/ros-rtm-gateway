#ifndef TEST_H_
#define TEST_H_
#include <iostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/any.hpp>

class Addict {
public:
	void memberAdd(int a, int b) {
		std::cout << "member function " << (a + b) << std::endl;
	}
	void (Addict::* func())(int,int) {
		return &Addict::memberAdd;
	}

	void printOut(std::string str){
		std::cout<< "printing: " << str << std::endl;
	}

	void printStg(){
		std::cout << "print something" << std::endl;
	}
};

struct X {
	void foo(int,int);
	void foo1(int);

	template<class T>
	void fooTemp(std::string str);
};


template<int c>
void addA(int a, int b) {
	std::cout << (a + b + c) << std::endl;
}

int addInts(int a, int b) {
	return a + b;
}

template<class t>
void addB(t a, int b) {
	std::cout << "*" << std::endl;
	a.memberAdd(b, b);
}

template<Addict& c>
void addC(int a, int b) {
	std::cout << "&" << std::endl;
	c.memberAdd(a, b);
}


int (*functionFactory(int n))(int, int) {
			//printf("Got parameter %d", n);
			int (*functionPtr)(int, int) = &addInts;
			return functionPtr;
		}

		void call(void (*fn)(int, int)) {
			fn(5, 6);
		}

#endif /* TEST_H_ */
