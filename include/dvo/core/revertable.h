#ifndef DVO_REVERTABLE_H
#define DVO_REVERTABLE_H

namespace dvo
{
	template<typename T>
	class Revertable
	{
	public:
		Revertable() :value()
		{

		}
		Revertable(const T& value) :value(value)
		{

		}
		inline const T& operator()() const
		{
			return value;
		}
		T& update()
		{
			old = value;
			return value;
		}
		void revert()
		{
			value = old;
		}
	private:
		T old, value;
	};
}
#endif