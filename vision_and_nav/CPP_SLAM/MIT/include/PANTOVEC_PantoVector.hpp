#ifndef PANTOVEC_PANTOVECTOR_HPP_
#define PANTOVEC_PANTOVECTOR_HPP_
#include <vector>
#include <queue>
#include <assert.h>
#include "CArenaAlloc.h"
#include "Config.hpp"

template<typename T>
class typePantoVector
{
private:
    template<bool IsConst>
        class BasicIterator
        {
            private:
                using OwnerType = std::conditional_t<
                    IsConst,
                    const typePantoVector,
                    typePantoVector
                        >;

                OwnerType* Owner;
                std::size_t Index;

                void skip_removed()
                {
                    while (Index < Owner->Data.size() &&
                            !Owner->Occupied[Index])
                    {
                        ++Index;
                    }
                }

            public:
                using iterator_category = std::forward_iterator_tag;
                using value_type        = T;
                using difference_type   = std::ptrdiff_t;

                using reference = std::conditional_t<
                    IsConst,
                    const T&,
                    T&
                        >;

                using pointer = std::conditional_t<
                    IsConst,
                    const T*,
                    T*
                        >;

                BasicIterator(OwnerType* Owner, std::size_t Index)
                    : Owner(Owner),
                    Index(Index)
            {
                skip_removed();
            }

                reference operator*() const
                {
                    return Owner->Data[Index];
                }

                pointer operator->() const
                {
                    return &Owner->Data[Index];
                }

                BasicIterator& operator++()
                {
                    ++Index;
                    skip_removed();
                    return *this;
                }

                BasicIterator operator++(int)
                {
                    BasicIterator Previous = *this;
                    ++(*this);
                    return Previous;
                }

                bool operator==(const BasicIterator& Other) const
                {
                    return Owner == Other.Owner &&
                        Index == Other.Index;
                }

                bool operator!=(const BasicIterator& Other) const
                {
                    return !(*this == Other);
                }

                std::size_t index() const
                {
                    return Index;
                }
        };

public:

    using size_type      = typename std::vector<T>::size_type;

    typePantoVector() = default;

    explicit typePantoVector(size_type Count)
        : Data(Count),
          Occupied(Count, true),
          ActiveCount(Count)
    {
    }

    typePantoVector(size_type Count, const T& Value)
        : Data(Count, Value),
          Occupied(Count, true),
          ActiveCount(Count)
    {
    }

    typePantoVector(std::initializer_list<T> Values)
        : Data(Values),
          Occupied(Values.size(), true),
          ActiveCount(Values.size())
    {
    }

    typePantoVector(const typePantoVector&) = default;
    typePantoVector(typePantoVector&&) noexcept = default;
    typePantoVector& operator=(const typePantoVector&) = default;
    typePantoVector& operator=(typePantoVector&&) noexcept = default;

    explicit typePantoVector(const std::vector<T>& Other)
        : Data(Other),
          Occupied(Other.size(), true),
          ActiveCount(Other.size())
    {
    }

    explicit typePantoVector(const std::vector<T>&& Other)
        : Data(Other),
          Occupied(Other.size(), true),
          ActiveCount(Other.size())
    {
    }

    typePantoVector& operator=(const std::vector<T>& Other)
    {
        Data = Other;
        Occupied.assign(Data.size(), true);
        FreeIDs = {};
        ActiveCount = Data.size();
        return *this;
    }

    size_type push_back(const T& Value)
    {
        if(FreeIDs.empty())
        {
            Data.push_back(Value);
            Occupied.push_back(true);
            ++ActiveCount;
            return Data.size() - 1;
        }

        const size_type Index = FreeIDs.front();
        FreeIDs.pop();

        Data[Index] = Value;
        Occupied[Index] = true;
        ++ActiveCount;
        return Index;
    }

    size_type push_back(T&& Value)
    {
        if (FreeIDs.empty())
        {
            Data.push_back(std::move(Value));
            Occupied.push_back(true);
            ++ActiveCount;
            return Data.size() - 1;
        }

        const size_type Index = FreeIDs.front();
        FreeIDs.pop();

        Data[Index] = std::move(Value);
        Occupied[Index] = true;
        ++ActiveCount;
        return Index;
    }

    void remove(size_type Index)
    {
        assert(Index < Data.size());
        assert(Occupied[Index]);

        Occupied[Index] = false;
        FreeIDs.push(Index);
        --ActiveCount;
    }

    bool contains(size_type Index) const
    {
        return Index < Data.size() && Occupied[Index];
    }

    T& operator[](size_type Index)
    {
        assert(contains(Index));
        return Data[Index];
    }

    const T& operator[](size_type Index) const
    {
        assert(contains(Index));
        return Data[Index];
    }

    size_type size() const
    {
        // Preserve vector-compatible index boundaries.
        return Data.size();
    }

    size_type active_size() const
    {
        return ActiveCount;
    }

    bool empty() const
    {
        return ActiveCount == 0;
    }

    void reserve(size_type Capacity)
    {
        Data.reserve(Capacity);
        Occupied.reserve(Capacity);
    }

    void clear()
    {
        Data.clear();
        Occupied.clear();
        FreeIDs = {};
        ActiveCount = 0;
    }

    T& back(void) 
    {
        if(Occupied.back())
        {
            return Data.back();
        }

        for(auto Iterator = Occupied.rbegin(); Iterator != Occupied.rend(); ++Iterator)
        {
            if(*Iterator)
            {
                return Data.back();
            }
        }

        assert(false);
    }

    T& front(void) 
    {
        if(Occupied.front())
        {
            return Data.front();
        }

        for(const bool Occup : Occupied)
        {
            if(Occup)
            {
                return Data.front();
            }
        }

        assert(false);
    }

    const T& back(void) const
    {
        if(Occupied.back())
        {
            return Data.back();
        }

        for(auto Iterator = Occupied.rbegin(); Iterator != Occupied.rend(); ++Iterator)
        {
            if(*Iterator)
            {
                return Data.back();
            }
        }

        assert(false);
    }

    const T& front(void) const
    {
        if(Occupied.front())
        {
            return Data.front();
        }

        for(const bool Occup : Occupied)
        {
            if(Occup)
            {
                return Data.front();
            }
        }

        assert(false);
    }

    void pop_back()
    {
        assert(!Data.empty());
        assert(Occupied.back() == true);

        Data.pop_back();
        Occupied.pop_back();
        --ActiveCount;
    }

    bool contains(u64 Index) const
    {
        return Occupied[Index];
    }

    T* data() noexcept
    {
        return Data.data();
    }

    const T* data() const noexcept
    {
        return Data.data();
    }

    operator std::vector<T>&() noexcept
    {
        return Data;
    }

    operator const std::vector<T>&() const noexcept
    {
        return Data;
    }

    using iterator = BasicIterator<false>;
    using const_iterator = BasicIterator<true>;

    iterator begin()
    {
        return iterator(this, 0);
    }

    iterator end()
    {
        return iterator(this, Data.size());
    }

    const_iterator begin() const
    {
        return const_iterator(this, 0);
    }

    const_iterator end() const
    {
        return const_iterator(this, Data.size());
    }

    const_iterator cbegin() const
    {
        return const_iterator(this, 0);
    }

    const_iterator cend() const
    {
        return const_iterator(this, Data.size());
    }

private:
    std::vector<T> Data;
    std::vector<bool> Occupied;
    std::queue<size_type> FreeIDs;
    size_type ActiveCount{0};
};

#endif //  PANTOVEC_PANTOVECTOR_HPP_
