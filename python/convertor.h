/*
 * @summary: recast convertor
 * @date: 2012-03-29
 * @author: zl
 */

#ifndef CONVERTOR_H_
#define CONVERTOR_H_

#include "config.h"
#include <list>
#include <vector>
#include <utility>

namespace converter = boost::python::converter;


template<class T>
struct list_to_list_converter {
	static PyObject* convert(const std::list<T>& v) {
		using boost::python::list;
		using boost::python::incref;

		list pylist;
		FOREACH(const T& e, v)
			pylist.append(e);
		return incref(pylist.ptr());
	}
};

template<class T>
struct vector_to_list_converter {
	static PyObject* convert(const std::vector<T>& v) {
		using boost::python::list;
		using boost::python::incref;

		list pylist;
		FOREACH(const T& e, v)
			pylist.append(e);
		return incref(pylist.ptr());
	}
};

template<class T1, class T2>
struct pair_to_tuple_converter {
	static PyObject* convert(const std::pair<T1, T2>& pair) {
		using boost::python::incref;
		using boost::python::make_tuple;

		return incref(make_tuple(pair.first, pair.second).ptr());
	}
};

template<typename T>
struct list_from_seq_converter {
	list_from_seq_converter() {
		using boost::python::type_id;
		converter::registry::push_back(&convertible, &construct, type_id<std::list<T> >());
	}

	static void* convertible(PyObject* obj_ptr) {
		// the second condition is important, for some reason otherwise there were attempted conversions of Body to list which failed afterwards.
		if(!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__"))
			return NULL;
		return obj_ptr;
	}

	static void construct(PyObject* obj_ptr, converter::rvalue_from_python_stage1_data* data) {
		using boost::python::handle;
		using boost::python::extract;

		void* storage = ((converter::rvalue_from_python_storage<std::list<T> >*)(data))->storage.bytes;
		new (storage) std::list<T>();
		std::vector<T>* v = (std::list<T>*)(storage);
		Py_ssize_t vlen = PySequence_Size(obj_ptr);
		if(vlen < 0)
			abort();
		v->reserve(vlen);
		for(int i = 0; i < vlen; i++) {
			handle<> item(PySequence_GetItem(obj_ptr, i));
			v->push_back(extract<T>(item.get()));
		}
		data->convertible = storage;
	}
};

template<typename T>
struct vector_from_seq_converter {
	vector_from_seq_converter() {
		using boost::python::type_id;
		converter::registry::push_back(&convertible, &construct, type_id<std::vector<T> >());
	}

	static void* convertible(PyObject* obj_ptr) {
		// the second condition is important, for some reason otherwise there were attempted conversions of Body to list which failed afterwards.
		if(!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__"))
			return NULL;
		return obj_ptr;
	}

	static void construct(PyObject* obj_ptr, converter::rvalue_from_python_stage1_data* data) {
		using boost::python::handle;
		using boost::python::extract;

		void* storage = ((converter::rvalue_from_python_storage<std::vector<T> >*)(data))->storage.bytes;
		new (storage) std::vector<T>();
		std::vector<T>* v = (std::vector<T>*)(storage);
		Py_ssize_t vlen = PySequence_Size(obj_ptr);
		if(vlen < 0)
			abort();
		v->reserve(vlen);
		for(int i = 0; i < vlen; i++) {
			handle<> item(PySequence_GetItem(obj_ptr, i));
			v->push_back(extract<T>(item.get()));
		}
		data->convertible = storage;
	}
};

struct dtvec3_from_seq_convertor {
	dtvec3_from_seq_convertor() {
		using boost::python::type_id;
		converter::registry::push_back(&convertible, &construct, type_id<dtVec3>());
	}

	static void* convertible(PyObject* obj_ptr) {
		if (!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__"))
			return 0;
		if (PySequence_Size(obj_ptr) != 3)
			return 0;
		return obj_ptr;
	}

	static void construct(PyObject* obj_ptr, converter::rvalue_from_python_stage1_data* data) {
		using boost::python::handle;
		using boost::python::extract;
		dtAssert(PySequence_Size(obj_ptr) == 3);

		void* storage = ((converter::rvalue_from_python_storage<dtVec3>*)(data))->storage.bytes;
		new (storage) dtVec3;
		dtVec3* v3 = (dtVec3*) storage;

		handle<> x(PySequence_GetItem(obj_ptr, 0));
		handle<> y(PySequence_GetItem(obj_ptr, 1));
		handle<> z(PySequence_GetItem(obj_ptr, 2));
		v3->x = extract<float>(x.get());
		v3->y = extract<float>(y.get());
		v3->z = extract<float>(z.get());
		data->convertible = storage;
	}
};


#endif /* CONVERTOR_H_ */
