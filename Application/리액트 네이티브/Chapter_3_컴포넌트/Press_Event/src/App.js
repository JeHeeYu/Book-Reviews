import { View, Text } from 'react-native';
import React from 'react';
import EventButton from './components/EventButton';

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
            <EventButton />
        </View>
    );
};

export default App;