/*
 * @summary: recast math
 * @date: 2013-03-20
 * @author: zl
 */

#include "config.h"
#include "dtmath.h"
#include "common.h"
#include "convertor.h"


using boost::python::object;


float frand_01() {
	using boost::random::mt19937;
	using boost::random::uniform_01;

	mt19937 rng(43);
	static uniform_01<mt19937> zeroone(rng);
	return (float) zeroone();
}

object dtVec3__str(dtVec3& self)
{
	using boost::format;
	using boost::python::str;
	return str((format("dtVec3(%.2f, %.2f, %.2f)") % self.x % self.y % self.z).str());
}

object dtVec3__iter(dtVec3& self)
{
	using boost::python::make_tuple;
	return make_tuple(self.x, self.y, self.z).attr("__iter__")();
}

float dtVec3__getitem(dtVec3& self, int index)
{
	if (index < -3 || index > 2) {
		IndexError();
		return 0.0f;
	}
	if (index < 0) {
		index += 3;
	}
	return (&self.x)[index];
}

void dtVec3__setitem(dtVec3& self, int index, float v)
{
	if (index < -3 || index > 2) {
		IndexError();
		return;
	}
	if (index < 0) {
		index += 3;
	}
	(&self.x)[index] = v;
}


void export_math()
{
	using namespace boost::python;

	class_<dtVec3>("dtVec3")
		.def(init<dtVec3>())
		.def(init<float, float, float>())
		.def("setZero", &dtVec3::setZero)
		.def("set", &dtVec3::set)
		.def_readwrite("x", &dtVec3::x)
		.def_readwrite("y", &dtVec3::y)
		.def_readwrite("z", &dtVec3::z)
		.def("dot", dtDot)
		.def("cross", dtCross)
		.def(-self)
		.def(self == other<dtVec3>())
		.def(self + other<dtVec3>())
		.def(self - other<dtVec3>())
		.def(self += other<dtVec3>())
		.def(self -= other<dtVec3>())
		.def(self * float())
		.def(self / float())
		.def(self *= float())
		.def(self /= float())
		.def(float() * self)
		.def(float() / self)
		.def("__iter__", &dtVec3__iter)
		.def("__getitem__", &dtVec3__getitem)
		.def("__setitem__", &dtVec3__setitem)
		.def("__str__", &dtVec3__str)
		.def("__repr__", &dtVec3__str)
	;

	vector_from_seq_converter<dtVec3>();
	to_python_converter<dtVec3List, vector_to_list_converter<dtVec3> >();
}

