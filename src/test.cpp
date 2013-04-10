/*
 * test.cpp
 *
 *  Created on: Apr 4, 2013
 *      Author: gbrj
 */

#include "test.h"
#include <vector>
#include "boost/any.hpp"

void f(int a, int b) {
	int r = a + b;
	std::cout << "2 param fn " << r << std::endl;
}

void g(int a, int b, int c) {
	int r = a + b + c;
	std::cout << "3 param fn " << r << std::endl;
}

typedef void (*MyFunc)(int, int);

void X::foo(int a, int b) {
	std::cout << "foo4 " << (a + b) << std::endl;
}

void X::foo1(int a) {
	std::cout << "foo1 " << a << std::endl;
}

template<class T>
void X::fooTemp(std::string str) {
	T t;
	t.printOut(str);
}

struct Num {
	int num;
};

struct Wrapper {
	Num num;
};

int main(int argc, char** argv) {
	std::vector<Wrapper> list;
	Wrapper w;
	int* numPtr = &w.num.num;
	list.push_back(w);
	*numPtr = 5;

	std::cout << "stack1: " << w.num.num << std::endl;
	std::cout << "stack2: " << list[0].num.num << std::endl;

	Wrapper& wrap = list[0];
	int& numRef = wrap.num.num;
	numRef = 8;

	std::cout << "stack3: " << w.num.num << std::endl;
	std::cout << "stack4: " << list[0].num.num << std::endl;

	list[0].num.num = 9;
	std::cout << "stack5: " << w.num.num << std::endl;
	std::cout << "stack6: " << list[0].num.num << std::endl;

	MyFunc f1 = &addA<7>;
	call(f1);

	addA<1>(2, 3);
	Addict t;
	addB<Addict>(t, 2);

	boost::function2<void, int, int> f2 = boost::bind(g, 1, _1, _2);
	f2(2, 3);

	boost::function3<void, X*, int, int> f3 = &X::foo;
	X x;
	boost::bind(f3, &x, 1, 2)();
	f3(&x, 5, 6);

	boost::function2<void, X*, int> f4 = &X::foo1;
	f4(&x, 8);

	std::vector<boost::any> l1;
	l1.push_back(f4);
	boost::function2<void, X*, int> f5 = boost::any_cast<boost::function2<void, X*, int> >(l1[0]);
	f5(&x, 15);
	//std::cout << "list: " << f5(&x, 15) << std::endl;

	x.fooTemp<Addict>("blah");

	boost::function2<void, X*, std::string> f6 = &X::fooTemp<Addict>;
	f6(&x, "hello");
	l1.push_back(f6);
	boost::function2<void, X*, std::string> f6c = boost::any_cast<boost::function2<void, X*, std::string> >(l1[1]);
	f6c(&x, "hello again");

}
