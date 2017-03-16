#include <iostream>
#include <boost/smart_ptr/shared_ptr.hpp>

using namespace std;

class  mat
{
public:
mat()
{
	data = 0;
};
mat(int b)
{
	data = b;
};

~mat()
{
	cout << "destroyed value  " << data << endl; 
};
private:

int data; 

};

int main()
{

	mat A(10);
	mat* B = new mat;
	B = &A;	

	boost::shared_ptr<double> a (new double);
	*a = 120;
	boost::shared_ptr<double> b;
	b = a;
	cout << "a original is " << *a << endl;
	a.reset();
//	b.reset();
	
	cout << "b's value is " << *b << endl;
	return 1;
	
}

