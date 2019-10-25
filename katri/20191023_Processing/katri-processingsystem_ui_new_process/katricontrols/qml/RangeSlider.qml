// RangeSlider QML
import QtQuick.Controls 2.5
import QtQuick 2.0



RangeSlider {
    id: _rangeSlider
    signal signalStartChange(double value)
    signal signalEndChange(double value)
    from: 0
    to: 1000
    first.value: 0
    second.value: 1000

    first.onMoved: _rangeSlider.signalStartChange(first.value)
    second.onMoved: _rangeSlider.signalEndChange(second.value)

    background: Rectangle {
        color: "#ff0000"

        Rectangle {
            x:0
            y:10
            width: 100
            height: 10
            color: "#ffffff"
        }
    }

}
