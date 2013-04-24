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

namespace converter = boost::python::converter;


template<class T>
struct list_to_list_converter {
	static PyObject* convert(const std::list<T>& v) {
		using namespace boost::python;
		list pylist; FOREACH(const T& e, v) pylist.append(e);
		return incref(pylist.ptr());
	}
};

template<class T>
struct vector_to_list_converter {
	static PyObject* convert(const std::vector<T>& v) {
		using namespace boost::python;
		list pylist; FOREACH(const T& e, v) pylist.append(e);
		return incref(pylist.ptr());
	}
};

template<typename T>
struct list_from_seq_converter {
	list_from_seq_converter() {
		using boost::python::type_id;
		converter::registry::push_back(&convertible, &construct, type_id<std::list<T> >());
	}

	static void* convertible(PyObject* obj_ptr){
		// the second condition is important, for some reason otherwise there were attempted conversions of Body to list which failed afterwards.
		if(!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__"))
			return NULL;
		return obj_ptr;
	}

	static void construct(PyObject* obj_ptr, converter::rvalue_from_python_stage1_data* data){
		using boost::python::extract;
		void* storage = ((converter::rvalue_from_python_storage<std::list<T> >*)(data))->storage.bytes;
		new (storage) std::list<T>();
		std::vector<T>* v = (std::list<T>*)(storage);
		int vlen = PySequence_Size(obj_ptr);
		if(vlen < 0)
			abort();
		v->reserve(vlen);
		for(int i = 0; i < vlen; i++) {
			v->push_back(extract<T>(PySequence_GetItem(obj_ptr, i)));
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

	static void* convertible(PyObject* obj_ptr){
		// the second condition is important, for some reason otherwise there were attempted conversions of Body to list which failed afterwards.
		if(!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__"))
			return NULL;
		return obj_ptr;
	}

	static void construct(PyObject* obj_ptr, converter::rvalue_from_python_stage1_data* data){
		using boost::python::extract;
		void* storage = ((converter::rvalue_from_python_storage<std::vector<T> >*)(data))->storage.bytes;
		new (storage) std::vector<T>();
		std::vector<T>* v = (std::vector<T>*)(storage);
		int vlen = PySequence_Size(obj_ptr);
		if(vlen < 0)
			abort();
		v->reserve(vlen);
		for(int i = 0; i < vlen; i++) {
			v->push_back(extract<T>(PySequence_GetItem(obj_ptr, i)));
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
		if (!PySequence_Check(obj_ptr))
			return 0;
		if (PySequence_Size(obj_ptr) != 3)
			return 0;
		return obj_ptr;
	}

	static void construct(PyObject* obj_ptr,
			converter::rvalue_from_python_stage1_data* data) {
		using boost::python::extract;
		dtAssert(PySequence_Size(obj_ptr) == 3);

		void* storage = ((converter::rvalue_from_python_storage<dtVec3>*)(data))->storage.bytes;
		new (storage) dtVec3;
		dtVec3* v3 = (dtVec3*) storage;

		v3->x = extract<float>(PySequence_GetItem(obj_ptr, 0));
		v3->y = extract<float>(PySequence_GetItem(obj_ptr, 1));
		v3->z = extract<float>(PySequence_GetItem(obj_ptr, 2));
		data->convertible = storage;
	}
};


#endif /* CONVERTOR_H_ */
