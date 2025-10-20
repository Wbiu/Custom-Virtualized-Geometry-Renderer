
#pragma once
#include <iostream>
#include <vector>
#include <engine/engine_std.h>

namespace engine::utils
{
	template<typename T, const size_t sizeOfArray>
	struct array
	{
		int insertAtIdx = 0;
		T data[sizeOfArray] = {};


		array() = default;
		array(std::initializer_list<T> init) {
			if (init.size() > sizeOfArray) throw std::out_of_range("array: too many initializers");
			insertAtIdx = 0;
			for (const T& v : init) data[insertAtIdx++] = v;
			for (size_t i = insertAtIdx; i < sizeOfArray; ++i) data[i] = T{};
		}

		void add(T& element)
		{

			if (insertAtIdx >= sizeOfArray) {
				throw std::out_of_range("ERROR::ENGINE::ARRAY::INSERT_OUT_OF_RANGE\n");
			}
			data[insertAtIdx] = element;
			insertAtIdx++;
		}

		/*
		zero could be a legitimate value!
		*/
		void compaction() {
			size_t writeIndex = 0;

			// First pass: move all non-zero values to the front
			for (size_t readIndex = 0; readIndex < sizeOfArray; readIndex++) {
				if (data[readIndex] != T{}) {  // Assuming T{} creates a "zero" value of type T
					if (writeIndex != readIndex) {
						data[writeIndex] = data[readIndex];
						data[readIndex] = T{};  // Clear the original position
					}
					writeIndex++;
				}
			}

			// Update insertAtIdx to point to the next available position
			insertAtIdx = writeIndex;
		}

		// Mutable access
		T& operator[](int atIdx)
		{
			return data[atIdx];
		}

		T operator[](int atIdx) const
		{
			return data[atIdx];
		}

		int size() const
		{
			return insertAtIdx;
		}
	
		void clear() noexcept 
		{
			// Reset all elements to their default value and reset the insert cursor.
			for (size_t i = 0; i < sizeOfArray; ++i) {
				data[i] = T{};
			}
			insertAtIdx = 0;
		}
	};

	template<typename T, typename Allocator>
	static int size_of(std::vector<T, Allocator> const& v)
	{
		return v.size() * sizeof(T);
	}


	inline void printMatrix(const engine::math::Mat4f& mat) {
		for (int row = 0; row < 4; ++row) {
			std::cout << "[ ";
			for (int col = 0; col < 4; ++col) {
				std::cout << mat[row][col] << " ";
			}
			std::cout << "]\n";
		}
	}


	inline void printMatrix(const engine::math::Mat3f& mat) {
		for (int row = 0; row < 3; ++row) {
			std::cout << "[ ";
			for (int col = 0; col < 3; ++col) {
				std::cout << mat[row][col] << " ";
			}
			std::cout << "]\n";
		}
	}

	inline void printVec(const engine::math::Vec4f& vec)
	{
		std::cout << "[" << vec.x << ","<< vec.y << "," << vec.z << "," << vec.w <<"]\n";
	}
	inline void printVec(const engine::math::Vec3f& vec)
	{
		std::cout << "[" << vec.x << "," << vec.y << "," << vec.z << "]\n";
	}


};