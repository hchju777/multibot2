#pragma once

#include <memory>
#include <vector>
#include <mutex>

namespace Observer
{
    template <typename Msg>
    class SubjectInterface;

    template <typename Msg>
    struct ObserverInterface
    {
    public:
        virtual ~ObserverInterface() {}
        virtual void update(const Msg &_msg) = 0;

    protected:
        std::mutex mtx_;
    }; // struct ObserverInterface

    template <typename Msg>
    struct SubjectInterface
    {
    public:
        virtual ~SubjectInterface() {}
        virtual void attach(Observer::ObserverInterface<Msg> &_observer) = 0;
        virtual void detach(Observer::ObserverInterface<Msg> &_observer) = 0;
        virtual void notify() = 0;

    protected:
        std::mutex mtx_;
    }; // struct SubjectInterface
} // namespace Observer