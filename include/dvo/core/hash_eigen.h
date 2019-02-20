#ifndef DVO_CORE_HASH_EIGEN_H
#define DVO_CORE_HASH_EIGEN_H
#include <utility>
#include <Eigen/Dense>
namespace dvo
{
namespace hash_tuple {

template <typename TT>
struct hash
{
	size_t operator()(TT const& tt) const
	{
		return std::hash<TT>()(tt);
	}
};

namespace {

	template <class T>
	inline void hash_combine(std::size_t& seed, T const& v)
	{
		seed ^= hash_tuple::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}

	template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1>
	struct HashValueImpl
	{
		static void apply(size_t& seed, Tuple const& tuple)
		{
			HashValueImpl<Tuple, Index - 1>::apply(seed, tuple);
			hash_combine(seed, std::get<Index>(tuple));
		}
	};

	template <class Tuple>
	struct HashValueImpl<Tuple, 0>
	{
		static void apply(size_t& seed, Tuple const& tuple)
		{
			hash_combine(seed, std::get<0>(tuple));
		}
	};

}    // unnamed namespace

template <typename ... TT>
struct hash<std::tuple<TT...>>
{
	size_t operator()(std::tuple<TT...> const& tt) const
	{
		size_t seed = 0;
		HashValueImpl<std::tuple<TT...> >::apply(seed, tt);
		return seed;
	}
};

}    // namespace hash_tuple

namespace hash_eigen {

	template <typename T>
	struct hash : std::unary_function<T, size_t> {
		std::size_t operator()(T const& matrix) const {
			size_t seed = 0;
			for (int i = 0; i < (int)matrix.size(); i++) {
				auto elem = *(matrix.data() + i);
				seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
					(seed << 6) + (seed >> 2);
			}
			return seed;
		}
	};

}    // namespace hash_eigen
}
#endif