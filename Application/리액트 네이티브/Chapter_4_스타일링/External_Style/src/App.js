import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { viewStyles, textStyles } from './styles';

const App = () => {
    return (
        <View style={viewStyles.container}>
            <Text style={[textStyles.text, { color: 'green' }]}>Text</Text>
            <Text style={[textStyles.text, textStyles.error]}>Error</Text>
        </View>
    );
};

export default App;