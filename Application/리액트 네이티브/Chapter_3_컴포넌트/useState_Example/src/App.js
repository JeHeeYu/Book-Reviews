import { View, Text } from 'react-native';
import React from 'react';
import CButton from './components/CButton';
import Counter from './components/Counter';

const App = () => {
    return (
        <View
            style={{
                flex: 1,
                backgroundColor: '#fff',
                alignItems: 'center',
                justifyContent: 'center',
            }}
        >
            <Counter />
        </View>
    );
};

export default App;