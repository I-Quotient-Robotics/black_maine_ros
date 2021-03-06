import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2

ApplicationWindow {
    id: applicationWindow
    width: 1280
    height: 720
    visible: true
    title: qsTr("AGV Panel")
    // visibility: "FullScreen"

    Material.theme: Material.Dark

    Button {
        id: confirm_button
        x: 651
        y: 448
        width: 558
        height: 110
        text: qsTr("Confirm")
        font.pointSize: 50
        autoExclusive: false

        onPressAndHold: {
            agv_bridge.confirm()
            console.info("confirm")
        }
    }

    Button {
        id: cancel_button
        x: 651
        y: 578
        width: 558
        height: 110
        text: qsTr("Cancel")
        font.pointSize: 50

        onPressAndHold: {
            agv_bridge.cancel()
            console.info("cancel")
        }
    }

    Image {
        id: logo_image
        width: 500
        height: 500

        anchors.verticalCenter: parent.verticalCenter
        anchors.verticalCenterOffset: 0
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: -307
        source: "../image/logo.svg"
        sourceSize.width: 500
        sourceSize.height: 500

        MouseArea {
            anchors.fill: parent

            onPressAndHold: {
                agv_bridge.locate()
                console.info("locate")
            }

            // onPressAndHold: {
            //     agv_bridge.reconnect()
            //     console.info("reconnect")
            // }
        }
    }

    Pane {
        id: pane
        x: 651
        y: 36
        width: 558
        height: 384

        Material.elevation: 6

        MouseArea {
            id: exit_mousearea
            anchors.fill: parent

            onPressAndHold: {
                Qt.quit()
            }
        }

        Column {
            id: column
            anchors.fill: parent
            spacing: 60

            Text {
                id: id_text
                text: qsTr("Truck 01")
                anchors.left: parent.left
                anchors.right: parent.right
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
                font.bold: true
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                font.pointSize: 50
                color: Material.color(Material.Grey)
            }

            Text {
                id: target_text
                text: qsTr("--")
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.left: parent.left
                anchors.right: parent.right
                wrapMode: Text.NoWrap
                font.pointSize: 75
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                color: Material.color(Material.primaryColor)
            }

            Text {
                id: log_text
                height: 50
                text: qsTr("nothing")
                elide: Text.ElideNone
                anchors.left: parent.left
                anchors.right: parent.right
                verticalAlignment: Text.AlignBottom
                wrapMode: Text.WordWrap
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                font.pointSize: 15
                color: Material.color(Material.Grey)
            }
        }
    }

    Connections {
        target: agv_bridge
        function onFeedbackUpdate(goal, state) {
            target_text.text = goal
            log_text.text = state
            console.log(goal, state)
        }

        function onResultUpdate(state) {
            log_text.text = state
            target_text.text = "--"
        }
    }

    Component.onCompleted: {
        console.log(agv_bridge.connected())
        log_text.text = agv_bridge.connected()?"Connected":"Disconnected"
    }
}

/*##^##
Designer {
    D{i:0;formeditorZoom:0.6600000262260437;height:720;width:1280}
}
##^##*/
