import { View, Text } from 'react-native';
import React from 'react';
import CButton from './components/CButton';

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
            <Text
                style={{
                    fontSize: 30,
                    marginBottom: 10,
                }}
            >
             Custom Button Component   
            </Text>
            <CButton />
        </View>
    );
};

export default App;