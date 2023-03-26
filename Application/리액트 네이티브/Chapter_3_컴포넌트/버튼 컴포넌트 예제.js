import { StatusBar } from 'react-native';
import {View, StyleSheet, Text, Button} from 'react-native';
import React from 'react';

const App = () => {
    const name = "A";
    return (
        <View 
            style={{
                flex: 1,
                backgroundColor: '#fff',
                alignItems: 'center',
                justifyContent: 'center',
            }}
        >
            <Text style={{ fontSize: 30, marginBottom: 10 }}>Button Component</Text>
            <Button title="Button" onPress={() => alert('Click !!!')} />
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        justifyContent: 'center',
        alignItems: 'center',
        backgroundColor: "#fff",
    },
    title: {
        fontSize: 30,
    },
});

export default App;
